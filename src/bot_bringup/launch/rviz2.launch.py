import os
import sys
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description(argv=sys.argv):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    bot_description_prefix = get_package_share_directory("bot_description")
    model = DeclareLaunchArgument(name="model", default_value=bot_description_prefix + "/urdf/bot.urdf.xacro")
    description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
    rviz_file = DeclareLaunchArgument(name="rviz_file", default_value=bot_description_prefix + "/rviz/bot_display.rviz")

    robot_state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description":description}]
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
            arguments=['-d', LaunchConfiguration('rviz_file')],
            parameters=[{'use_sim_time': use_sim_time}],
            )

    if len(argv) == 4:
        return LaunchDescription([model, rviz_file, rviz2, robot_state_pub, joint_state_pub])
    else:
        return LaunchDescription([model, rviz_file, rviz2, robot_state_pub])
        # ros2 launch bot_bringup rviz2.launch.py rviz_file:=$(ros2 pkg prefix bot_description)/share/bot_description/rviz/bot_gazebo.rviz
