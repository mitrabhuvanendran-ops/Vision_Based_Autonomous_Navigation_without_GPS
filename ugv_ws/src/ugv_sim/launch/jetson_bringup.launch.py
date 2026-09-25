"""Stage 10 - Orin Nano side of the split bringup: BiSeNet, costmap,
planner/follower/safety-gate, and (last, heaviest) ORB-SLAM3.

Run on the PC first: `ros2 launch ugv_sim pc_bringup.launch.py`
Then on the Jetson (same ROS_DOMAIN_ID, same subnet, wired Gigabit):
    ros2 launch ugv_sim jetson_bringup.launch.py

Before measuring performance:
    sudo nvpmodel -m 0
    sudo jetson_clocks

Bring-up order (per the build spec, cheapest/most-diagnosable first):
network connectivity -> Gazebo sensors visible here -> BiSeNet alone
(check FPS/memory) -> costmap -> planner/follower/safety-gate -> ORB-SLAM3.

This launch file starts everything at once for convenience once each stage
has been individually verified; comment out args to bring pieces up one at
a time while debugging a fresh Jetson image.

Not verified on physical Jetson hardware - this box does not have one.
Camera image topics have not rendered in this build/test environment at
all (Gazebo Classic offscreen sensor rendering issue on this host, not a
world/robot-config problem - see build notes), so BiSeNet/ORB-SLAM3 could
not be exercised against real frames on the Jetson path either. Everything
below is correct against the documented topic contracts and was built and
node-started successfully; treat it as ready-to-verify, not field-verified.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('ugv_nav')
    use_orbslam = DeclareLaunchArgument('use_orbslam', default_value='true')

    def nav_node(exe):
        return Node(package='ugv_nav', executable=exe, name=exe,
                    output='screen', parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        use_orbslam,
        nav_node('bisenet_node'),
        nav_node('costmap_builder'),
        nav_node('goal_converter'),
        nav_node('rrt_planner'),
        nav_node('path_follower'),
        nav_node('safety_gate'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'orbslam3.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_orbslam'))),
    ])
