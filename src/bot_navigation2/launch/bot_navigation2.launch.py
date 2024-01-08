import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_file = os.path.join(get_package_share_directory('bot_cartographer'), 'map', 'bot_cartographer_ok.yaml')
    param_file = os.path.join(get_package_share_directory('bot_navigation2'), 'config', 'bot_nav_params.yaml')
    rviz_file = os.path.join(get_package_share_directory('bot_navigation2'), 'rviz', 'bot_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'param_file': param_file
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])