#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging


def generate_launch_description():
    ld = LaunchDescription()
    
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Names and poses of the robots
    robots = [
        {'name': 'tb1', 'x_pose': '-4', 'y_pose': '4', 'z_pose': '0.01'},
        {'name': 'tb2', 'x_pose': '-1', 'y_pose': '1', 'z_pose': '0.01'},
        #{'name': 'tb3', 'x_pose': '-1', 'y_pose': '4', 'z_pose': 0.01},
        #{'name': 'tb3', 'x_pose': '-4', 'y_pose': '1', 'z_pose': 0.01},
        
    ]
    TURTLEBOT3_MODEL = 'burger'
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    turtlebot3_simulations = get_package_share_directory('turtlebot3_simulations')
    
    # Use the house world
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )
 
    
    # Get the urdf file for spawn
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    #State publiser udrf description
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path_state = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    with open(urdf_path_state, 'r') as infp:
        robot_desc = infp.read()
    
    # Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )


    # Add Gazebo launch commands
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    #remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    ######################

    last_action = None
    
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:
    
        namespace = [ '/' + robot['name'] ]
        
        # Create state publisher node for that instance 
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        )

        # Create spawn call   
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-file', urdf_path,
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', '0.01',
                # '-Y', '0.0',
                # '-unpause',
            ],
            output='screen',
        )
        
        #Ensure the next robot starts to spawn until the previous is completely spawned 
        if last_action is None:
            # Launch first robot inmediately
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
        else:
            # Subsequent robots, wait for the previous to finish
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger,
                            turtlebot_state_publisher],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next the event handler
        last_action = spawn_turtlebot3_burger
    ######################
        
        
    return ld


