#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the simulation launch file
    interbotix_xslocobot_sim_dir = get_package_share_directory('interbotix_xslocobot_sim')
    simulation_launch_path = os.path.join(interbotix_xslocobot_sim_dir, 'launch', 'xslocobot_gz_classic.launch.py')

    # Simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={'robot_model': 'locobot_wx200'}.items()
    )

    # Node execution with parameters
    control_node = Node(
        package='locobot_base_control',
        executable='circular_control.py',
        name='circular_control',
        parameters=[{'Kp_linear': 0.5}]
    )

    # delay
    delay_and_conotrol = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[control_node]
    )

    # Start rqt_graph
    rqt_graph = ExecuteProcess(
        cmd=['rqt_graph'],
        shell=True
    )

    # Uncomment to enable ROS bag recording
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/rosbag','/locobot/position_stamped']
    )
    # Stop recording after 20 seconds
    stop_record_bag = TimerAction(
        period=20.0,  # 20 seconds
        actions=[Shutdown()]
    )

    echo_bag = ExecuteProcess(
        cmd=['ros2 topic echo /locobot/twist_stamped > /path/to/kp_05_data.csv']
    )
    return LaunchDescription([
        simulation_launch,
        delay_and_conotrol
        # control_node
        # rqt_graph,
        # record_bag,
        # stop_record_bag
        # play_bag
        # echo_bag
    ])
