from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node'
        ),
        ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='training',
                    plugin='composition::TurtleSpawner',
                    name='turtle_spawner'),
                ComposableNode(
                    package='training',
                    plugin='composition::TurtleClearer',
                    name='turtle_clearer'),
                ComposableNode(
                    package='training',
                    plugin='composition::CircularMotion',
                    name='circular_motion'),
                ComposableNode(
                    package='training',
                    plugin='composition::DistancePublisher',
                    name='dist_pub'),
                ComposableNode(
                    package='training',
                    plugin='composition::TurtleMover',
                    name='turtle_mover'),
                ComposableNode(
                    package='training',
                    plugin='composition::TurtleReset',
                    name='turtle_reset')
            ]
        )
    ])