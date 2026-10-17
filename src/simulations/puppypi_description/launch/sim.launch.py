import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_path = get_package_share_directory("puppypi_description")
    gazebo_ros_path = get_package_share_directory("gazebo_ros")

    world_path = os.path.join(pkg_path, "worlds", "puppy_room.world")
    xacro_path = os.path.join(pkg_path, "urdf", "puppy_sim.xacro")


    # robot_description = ParameterValue(Command(['xacro ', xacro_path, ' fixed_legs:=true']), value_type=str)
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)


    model_path = AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.dirname(pkg_path))


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path, 'pause': 'false', 'verbose': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'puppypi', '-z', '0.15'])

    return LaunchDescription([model_path, gazebo, robot_state_publisher, spawn])

