#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'racecar.world'
    world = os.path.join(get_package_share_directory('racecar_description'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('racecar'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world, 'verbose':'error'}.items(),
        )

    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        )

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'], output='screen'
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
        )

    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
        )
    
    spawn_entity = Node(
        name='spawn_robot',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'racecar',
            '-topic',  '/robot_description',
        ]
    )
        
    user_controller = Node(
        package = "racecar",
        executable = "control"
        )

    nodes = [
        gzserver,
        gzclient, 
        spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster,
        forward_position_controller,
        forward_velocity_controller,
        user_controller,
        #Added by siddhesh
        Node(
            name="vehicle_model",
            package="racecar",
            executable="vehicle_model",
            output="screen",

        ),
        # Node(
        #     name="teleop_twist_keyboard",
        #     package="teleop_twist_keyboard",
        #     executable="teleop_twist_keyboard",
        #     output="screen",
        # )
    ]

    return LaunchDescription(nodes)