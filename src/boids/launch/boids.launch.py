"""Launch N boids in one command.

Usage:
    ros2 launch boids boids.launch.py
    ros2 launch boids boids.launch.py num_boids:=20
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_boids(context):
    """Build the list of Node actions to spawn, one per boid."""
    # Read the launch argument's current value.
    # context.perform_substitution turns the symbolic LaunchConfiguration
    # into an actual string we can convert to int.
    n = int(LaunchConfiguration('num_boids').perform(context))

    nodes = []
    for i in range(n):
        nodes.append(Node(
            package='boids',
            executable='single_boid',
            name=f'boid_{i}',                  # unique node name in ros2 node list
            parameters=[{
                'boid_id': i,
                'num_boids': n,
            }],
            output='screen',                   # show this node's logs in the terminal
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_boids',
            default_value='4',
            description='How many boids to spawn',
        ),
        OpaqueFunction(function=spawn_boids),
    ])