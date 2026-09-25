import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('ugv_nav')

    # The orbslam3_ros2 wrapper's own config/ dir does not actually contain
    # ORBvoc.txt (its default launch file assumes it does and is wrong too);
    # the real ~140MB vocabulary lives in the vendored ORB_SLAM3 checkout.
    vocab_file = os.path.expanduser('~/ugv_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    settings_file = os.path.join(nav_pkg, 'config', 'orbslam3_gazebo_rgbd.yaml')

    return LaunchDescription([
        # Stage 5.2 - virtual map + drift comparison only; the EKF (Stage 5.1)
        # is what actually drives the robot. Remapped onto the sim's real
        # co-located RGB-D topics (Stage 2 camera-alignment fix).
        Node(
            package='orbslam3_ros2',
            executable='rgbd',
            name='orbslam3_rgbd',
            output='screen',
            parameters=[
                {'vocab_path': vocab_file},
                {'config_path': settings_file},
                {'use_sim_time': True},
            ],
            remappings=[
                ('/camera/rgb/image_color', '/camera/rgb_camera/image_raw'),
                ('/camera/depth/image', '/camera/depth/depth/image_raw'),
                ('/slam/odometry', '/orb/odometry'),
                ('/slam/pointcloud', '/orb/map_points'),
            ],
        ),
    ])
