import os

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    model = DeclareLaunchArgument(name="model", default_value=get_package_share_directory("bot_description") + "/urdf/bot.urdf.xacro")
    p_value = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
    world_file = os.path.join(
        get_package_share_directory('bot_description'), 'world', 'bot.world'
    )
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',world_file
        ],
        output='screen'
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":p_value}]
    )

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'III-Robot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        model,
        robot_state_pub,
        joint_state_pub,
        spawn_entity
    ])