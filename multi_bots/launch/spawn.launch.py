import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    # Robot parameters
    x_spawn = LaunchConfiguration('x_spawn').perform(context)
    y_spawn = LaunchConfiguration('y_spawn').perform(context)
    entity_name = LaunchConfiguration('entity_name').perform(context)

    print("###############################################################")
    print("SPAWN MULTI Robot="+str(entity_name)+",["+str(x_spawn)+","+str(y_spawn)+"]")
    print("###############################################################")

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_path = os.path.join(get_package_share_directory('multi_bots'))
    xacro_file = os.path.join(pkg_path, "description", 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' robot_name:=', entity_name])

    params = {'robot_description': robot_description_config, 'frame_prefix': entity_name+'/', 'use_sim_time': use_sim_time}
    
    # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=entity_name,
        parameters=[params],
        output='screen',
    )
    
    # Create a joint_state_publisher node
    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=entity_name,
        parameters=[{'robot_description': robot_description_config, 'frame_prefix': entity_name+'/', 'use_sim_time': use_sim_time}],
        output="screen"
    )

    # package_name='multi_bots'

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped/' + entity_name)]
    #     )

    # Run the spawner node from the gazebo_ros package.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        name='spawn_entity',
                        namespace=entity_name,
                        arguments=['-topic', 'robot_description',
                                   '-entity', entity_name,
                                   '-x', x_spawn, '-y', y_spawn,
                                   '-timeout', '120.0'
                                   ],
                        output='screen')

    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",    
    #     arguments=["diff_cont"],
    # )


    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )


    return [
        robot_state_publisher_node,
        joint_state_publisher_node, 
        # twist_mux,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner
        ]


def generate_launch_description(): 

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_spawn', default_value='1.0'),
        DeclareLaunchArgument('y_spawn', default_value='2.0'),
        DeclareLaunchArgument('entity_name', default_value='walle'),

        OpaqueFunction(function = launch_setup)
        ])