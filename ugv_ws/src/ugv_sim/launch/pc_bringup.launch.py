"""Stage 10 - PC side of the split bringup: Gazebo, robot, EKF (cheap,
always-on, stays co-located with its /odom + /imu sources), and the
dashboard. Perception/planning/control move to the Orin Nano via
jetson_bringup.launch.py. Both machines must share ROS_DOMAIN_ID and subnet
(wired Gigabit recommended - see Stage 10 notes in the build spec)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_robot = get_package_share_directory('ugv_robot')
    pkg_nav = get_package_share_directory('ugv_nav')
    pkg_web = get_package_share_directory('ugv_web')

    use_dashboard = DeclareLaunchArgument('use_dashboard', default_value='true')

    return LaunchDescription([
        use_dashboard,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'forest_robot.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'ekf.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_web, 'launch', 'dashboard.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_dashboard'))),
    ])
