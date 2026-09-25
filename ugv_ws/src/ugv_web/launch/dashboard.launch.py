import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    www_dir = os.path.join(get_package_share_directory('ugv_web'), 'www')

    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}],
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=www_dir,
            output='screen',
        ),
    ])
