import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot = get_package_share_directory('ugv_robot')
    pkg_nav = get_package_share_directory('ugv_nav')
    pkg_web = get_package_share_directory('ugv_web')

    use_orbslam = DeclareLaunchArgument(
        'use_orbslam', default_value='true',
        description='Launch ORB-SLAM3 (Stage 5.2, virtual map only - EKF drives the robot)')
    use_dashboard = DeclareLaunchArgument(
        'use_dashboard', default_value='true',
        description='Launch rosbridge + web dashboard (Stage 9)')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'forest_robot.launch.py')))

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'ekf.launch.py')))

    orbslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'orbslam3.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_orbslam')))

    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_web, 'launch', 'dashboard.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_dashboard')))

    def nav_node(exe, name=None):
        return Node(package='ugv_nav', executable=exe, name=name or exe,
                    output='screen', parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        use_orbslam,
        use_dashboard,
        sim,
        ekf,
        orbslam,
        dashboard,
        nav_node('bisenet_node'),
        nav_node('costmap_builder'),
        nav_node('goal_converter'),
        nav_node('rrt_planner'),
        nav_node('path_follower'),
        nav_node('safety_gate'),
    ])
