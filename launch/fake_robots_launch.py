#!/usr/bin/env python3

import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    Setup function that gets called with the launch context.
    This allows us to evaluate LaunchConfiguration values.
    """
    # Get launch configuration values and convert to appropriate types
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    env_size = int(LaunchConfiguration('env_size').perform(context))
    speed_limit = LaunchConfiguration('speed_limit')  # Keep as LaunchConfiguration for parameter
    ns_prefix = LaunchConfiguration('ns_prefix').perform(context)
    buffer_size = int(LaunchConfiguration('buffer_size').perform(context))
    
    # Create list to hold robot nodes
    robot_nodes = []
    
    # Generate nodes for each robot
    for i in range(1, num_robots + 1):
        # Generate random positions within bounds (as floats)
        pos_x = float(random.uniform(buffer_size, env_size - buffer_size))
        pos_y = float(random.uniform(buffer_size, env_size - buffer_size))
        
        # Create robot node
        robot_node = Node(
            package='gcs',
            executable='fake_robot',
            namespace=f'{ns_prefix}{i}',
            name='fake_robot_node',
            parameters=[{
                'pos_x': pos_x,
                'pos_y': pos_y,
                'speed_limit': speed_limit
            }]
        )
        
        robot_nodes.append(robot_node)
    
    return robot_nodes


def generate_launch_description():
    # Declare launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        description='Number of robots to spawn'
    )
    
    env_size_arg = DeclareLaunchArgument(
        'env_size',
        default_value='250',
        description='Environment size (default: 250)'
    )
    
    speed_limit_arg = DeclareLaunchArgument(
        'speed_limit',
        default_value='2.0',
        description='Speed limit for robots (default: 2.0)'
    )
    
    ns_prefix_arg = DeclareLaunchArgument(
        'ns_prefix',
        default_value='fr',
        description='Namespace prefix for robots (default: fr)'
    )
    
    buffer_size_arg = DeclareLaunchArgument(
        'buffer_size',
        default_value='10',
        description='Buffer size for position generation (default: 10)'
    )
    
    # Use OpaqueFunction to handle dynamic node creation
    opaque_function = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        num_robots_arg,
        env_size_arg,
        speed_limit_arg,
        ns_prefix_arg,
        buffer_size_arg,
        opaque_function
    ])

################################################################################
# Basic usage 
# ros2 launch fake_robots_launch.py num_robots:=3

# With all optional arguments
# ros2 launch fake_robots_launch.py num_robots:=5 env_size:=300 speed_limit:=1.5 ns_prefix:=robot buffer_size:=15
################################################################################
