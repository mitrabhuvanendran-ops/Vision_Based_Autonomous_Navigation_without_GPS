import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('ugv_robot')
    pkg_terrain = get_package_share_directory('ugv_terrain')

    world_file = os.path.join(pkg_terrain, 'worlds', 'forest.world')
    xacro_file = os.path.join(pkg_robot, 'urdf', 'ugv.urdf.xacro')

    models_dir = os.path.join(pkg_terrain, 'models')
    textures_dir = os.path.join(pkg_terrain, 'materials', 'textures')
    scripts_dir = os.path.join(pkg_terrain, 'materials', 'scripts')

    # gazebo_ros's own gazebo.launch.py does NOT source /usr/share/gazebo/setup.sh,
    # so GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH are typically empty unless the shell
    # already sourced it. Fall back to Gazebo's default install paths explicitly so
    # we don't strip out core Ogre media (shaders) and default models (e.g. "sun"),
    # which crashes the render engine if missing.
    default_gazebo_share = '/usr/share/gazebo-11'

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '') or f"{default_gazebo_share}/models"
    combined_model_path = f"{models_dir}:{existing_model_path}"

    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '') or default_gazebo_share
    combined_resource_path = f"{pkg_terrain}:{textures_dir}:{scripts_dir}:{existing_resource_path}"

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', combined_model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', combined_resource_path),

        # Standard Gazebo ROS launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file, 'verbose': 'true'}.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # Robot Spawner with a 60-second service wait timeout
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_ugv',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'ugv_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.35',
                '-Y', '0.0',
                '-timeout', '60'
            ],
            output='screen'
        )
    ])
