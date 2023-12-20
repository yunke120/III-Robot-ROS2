import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    model = DeclareLaunchArgument(name="model", default_value=get_package_share_directory("bot_description") + "/urdf/bot.urdf.xacro")
    description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    world_file = os.path.join(
        get_package_share_directory('bot_description'), 'world', 'bot.world'
    )

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
    
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',world_file
        ],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'III-Robot', '-topic', 'robot_description'],
        output='screen'
    )

    delay_rviz2_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[rviz2],
        )
    )

    return LaunchDescription([model, 
                            #   joint_state_pub, # 如果publishWheelTF为true，需关闭joint_state_pub，防止重复发布
                              robot_state_pub, 
                              gazebo, 
                              spawn_entity, 
                              rviz2])