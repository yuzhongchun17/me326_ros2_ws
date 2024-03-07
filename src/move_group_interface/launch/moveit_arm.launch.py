from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Node setup
    moveit_arm_node = Node(
        package='move_group_interface',
        executable='moveit_arm',
        name='moveit_arm',
        output='screen',
        parameters=[{'use_sim_time': True}],
        
    )

    # Launch Description
    return LaunchDescription([
        moveit_arm_node,
    ])
