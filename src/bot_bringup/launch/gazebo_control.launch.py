import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

# $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

def generate_launch_description():

    model = DeclareLaunchArgument(name="model", default_value=get_package_share_directory("bot_description") + "/urdf/bot.urdf.xacro")
    p_value = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description":p_value}]
    )

    # joint_state_pub = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    # )

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

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller"]
    )

    return LaunchDescription([model, robot_state_pub, rviz2, gazebo, spawn_entity,joint_broad_spawner,diff_drive_spawner])