#!/usr/bin/env python3
"""Stage 7.3 - Goal-biased, obstacle-aware RRT* over /costmap.

"RGO-RRT" could not be confirmed as a published, citable algorithm, so per
the build spec this implements (and improves on) the documented fallback:
RRT* - Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion
Planning" (2011) - rather than plain RRT. Each new node picks the
lowest-cost collision-free parent among its neighbors (not just the
nearest), and nearby nodes get rewired through it if that lowers their
cost, so the tree keeps improving path quality within the time budget
instead of stopping at the first feasible route. Goal-biased sampling,
inflated-costmap collision checking, a node/time budget with
best-partial-path fallback, and shortcut smoothing are all retained.
Replanned continuously as the costmap updates.
"""
import math
import random
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.ndimage import grey_dilation

UNKNOWN, OCCUPIED_THRESH = -1, 50


class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('goal_bias', 0.15)
        self.declare_parameter('step_size', 1.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('max_nodes', 1200)
        self.declare_parameter('time_budget_s', 0.2)
        self.declare_parameter('robot_radius', 0.4)
        self.declare_parameter('replan_rate', 4.0)
        self.declare_parameter('rewire_radius', 2.0)

        self.goal_bias = self.get_parameter('goal_bias').value
        self.step = self.get_parameter('step_size').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.max_nodes = self.get_parameter('max_nodes').value
        self.time_budget = self.get_parameter('time_budget_s').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.rewire_radius = self.get_parameter('rewire_radius').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.costmap = None
        self.goal = None
        self.create_subscription(OccupancyGrid, '/costmap', self.costmap_cb, 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.plan_pub = self.create_publisher(Path, '/plan', 10)

        self.create_timer(1.0 / self.get_parameter('replan_rate').value, self.replan)
        self.get_logger().info("RRT planner active")

    def costmap_cb(self, msg):
        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        occ = (grid >= OCCUPIED_THRESH).astype(np.uint8)
        inflate_cells = max(1, int(math.ceil(self.robot_radius / msg.info.resolution)))
        occ_inflated = grey_dilation(occ, size=(2 * inflate_cells + 1, 2 * inflate_cells + 1))
        self.costmap = {
            'occ': occ_inflated,
            'res': msg.info.resolution,
            'ox': msg.info.origin.position.x,
            'oy': msg.info.origin.position.y,
            'w': msg.info.width,
            'h': msg.info.height,
        }

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def world_to_cell(self, x, y, cm):
        return (int((x - cm['ox']) / cm['res']), int((y - cm['oy']) / cm['res']))

    def is_free(self, x, y, cm):
        gx, gy = self.world_to_cell(x, y, cm)
        if gx < 0 or gx >= cm['w'] or gy < 0 or gy >= cm['h']:
            return True  # treat unknown/out-of-sensor-range as free (Stage 7.3)
        return cm['occ'][gy, gx] == 0

    def segment_free(self, p1, p2, cm):
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        n_checks = max(1, int(dist / (cm['res'] * 1.5)))
        for i in range(n_checks + 1):
            t = i / n_checks
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self.is_free(x, y, cm):
                return False
        return True

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.1))
            return (trans.transform.translation.x, trans.transform.translation.y)
        except Exception:
            return None

    def plan_rrt(self, start, goal, cm):
        """RRT* (Karaman & Frazzoli 2011): each new node picks the lowest-cost
        collision-free parent among nearby nodes, then rewires nearby nodes
        through it if that shortens their path - so path quality keeps
        improving for as long as the time budget allows, not just "first
        found, first used" like plain RRT."""
        nodes = np.empty((self.max_nodes, 2), dtype=np.float64)
        nodes[0] = start
        n = 1
        parents = [-1]
        costs = [0.0]

        deadline = time.time() + self.time_budget
        best_idx, best_dist = 0, math.hypot(start[0] - goal[0], start[1] - goal[1])

        while n < self.max_nodes and time.time() < deadline:
            if random.random() < self.goal_bias:
                sample = goal
            else:
                sample = (
                    cm['ox'] + random.random() * cm['w'] * cm['res'],
                    cm['oy'] + random.random() * cm['h'] * cm['res'],
                )

            deltas = nodes[:n] - np.array(sample)
            sq_dists = np.einsum('ij,ij->i', deltas, deltas)
            nearest_idx = int(np.argmin(sq_dists))
            nearest = tuple(nodes[nearest_idx])
            dist = math.sqrt(sq_dists[nearest_idx])
            if dist < 1e-6:
                continue
            scale = min(self.step, dist) / dist
            new_pt = (nearest[0] + (sample[0] - nearest[0]) * scale,
                      nearest[1] + (sample[1] - nearest[1]) * scale)

            # Neighbors within rewire_radius, cheapest-cost-first, so the
            # first collision-free candidate found is already the best parent.
            near_deltas = nodes[:n] - np.array(new_pt)
            near_dists = np.sqrt(np.einsum('ij,ij->i', near_deltas, near_deltas))
            near_mask = near_dists <= self.rewire_radius
            near_mask[nearest_idx] = True
            near_idxs = np.nonzero(near_mask)[0]
            near_costs = np.array([costs[i] for i in near_idxs]) + near_dists[near_idxs]
            order = np.argsort(near_costs)

            chosen_parent, chosen_cost = None, None
            for k in order:
                cand_idx = int(near_idxs[k])
                cand_pt = tuple(nodes[cand_idx])
                if self.segment_free(cand_pt, new_pt, cm):
                    chosen_parent, chosen_cost = cand_idx, float(near_costs[k])
                    break
            if chosen_parent is None:
                continue

            new_idx = n
            nodes[new_idx] = new_pt
            parents.append(chosen_parent)
            costs.append(chosen_cost)
            n += 1

            # Rewire: does routing existing neighbors through the new node
            # shorten their path?
            for k in order:
                cand_idx = int(near_idxs[k])
                if cand_idx == chosen_parent:
                    continue
                via_new_cost = chosen_cost + float(near_dists[k])
                if via_new_cost < costs[cand_idx] - 1e-6:
                    cand_pt = tuple(nodes[cand_idx])
                    if self.segment_free(new_pt, cand_pt, cm):
                        parents[cand_idx] = new_idx
                        costs[cand_idx] = via_new_cost

            d_to_goal = math.hypot(new_pt[0] - goal[0], new_pt[1] - goal[1])
            if d_to_goal < best_dist:
                best_dist, best_idx = d_to_goal, new_idx
            # Unlike plain RRT, don't stop at first contact with the goal
            # region - keep refining (lower-cost rewires) until the time
            # budget runs out, per RRT*'s asymptotic-optimality property.

        path = []
        idx = best_idx
        while idx != -1:
            path.append(tuple(nodes[idx]))
            idx = parents[idx]
        path.reverse()
        return path

    def smooth(self, path, cm):
        if len(path) < 3:
            return path
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.segment_free(path[i], path[j], cm):
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    def replan(self):
        if self.costmap is None or self.goal is None:
            return
        start = self.get_current_pose()
        if start is None:
            return

        dist_to_goal = math.hypot(start[0] - self.goal[0], start[1] - self.goal[1])
        if dist_to_goal < self.goal_tol:
            # Goal reached cleanly! Publish empty path and clear goal to stop robot right there.
            self.get_logger().info("Target destination reached successfully! Stopping UGV.")
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'odom'
            self.plan_pub.publish(empty_path)
            self.goal = None
            return

        cm = dict(self.costmap)
        cm['occ'] = cm['occ'].copy()
        sgx, sgy = self.world_to_cell(start[0], start[1], cm)
        if 0 <= sgx < cm['w'] and 0 <= sgy < cm['h']:
            cm['occ'][max(0, sgy-2):min(cm['h'], sgy+3), max(0, sgx-2):min(cm['w'], sgx+3)] = 0

        raw_path = self.plan_rrt(start, self.goal, cm)
        if len(raw_path) < 2:
            return
        path = self.smooth(raw_path, cm)

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = Point(x=x, y=y, z=0.0)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.plan_pub.publish(msg)


def main():
    rclpy.init()
    node = RRTPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
