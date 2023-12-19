import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    model = DeclareLaunchArgument(name="model", default_value=get_package_share_directory("bot_description") + "/urdf/bot.urdf.xacro")
    description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}]
    )

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', get_package_share_directory("bot_description") + "/rviz/bot.rviz"])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )])
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'III-Robot', '-topic', 'robot_description'],
        output='screen'
    )



    return LaunchDescription([model, joint_state_pub, robot_state_pub, rviz2, gazebo, spawn_entity])