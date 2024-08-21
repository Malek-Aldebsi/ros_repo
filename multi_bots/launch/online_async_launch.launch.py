import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bot_v0_slam_params_file = os.path.join(get_package_share_directory("multi_bots"), 'config', 'bot_v0_mapper_params_online_async.yaml')
    bot_v1_slam_params_file = os.path.join(get_package_share_directory("multi_bots"), 'config', 'bot_v1_mapper_params_online_async.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    start_async_slam_toolbox_node_bot_v0 = Node(
        parameters=[
          bot_v0_slam_params_file,
          {'use_sim_time': use_sim_time},
        ],
        remappings=[('/map', '/map1')],  # Add the remapping here
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    start_async_slam_toolbox_node_bot_v1 = Node(
        parameters=[
          bot_v1_slam_params_file,
          {'use_sim_time': use_sim_time},
        ],
        remappings=[('/map', '/map2')],  # Add the remapping here
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_async_slam_toolbox_node_bot_v0)
    ld.add_action(start_async_slam_toolbox_node_bot_v1)

    return ld
