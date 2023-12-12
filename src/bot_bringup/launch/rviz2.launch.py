import os

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    model = DeclareLaunchArgument(name="model", default_value=get_package_share_directory("bot_description") + "/urdf/bot.urdf.xacro")
    p_value = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description":p_value}]
    )

    joint_state_pub = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', get_package_share_directory("bot_description") + "/rviz/bot.rviz"])


    return LaunchDescription([model, rviz2, robot_state_pub, joint_state_pub])
    