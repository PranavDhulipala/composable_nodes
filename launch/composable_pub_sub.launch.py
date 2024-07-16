""" ROS 2 launch file for running a composable node container with a publisher and subscriber component. """

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generate launch description for a ROS 2 launch file.

    This function sets up a container for running composable ROS 2 nodes,
    a publisher and a subscriber component, within the same process
    using multi-threaded execution.

    Returns:
        LaunchDescription: The launch description object that includes the
        composable node container with the publisher and subscriber components.
    """
    container = ComposableNodeContainer(
        name="components_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # Use multi-threaded execution by default
        composable_node_descriptions=[
            ComposableNode(
                package="composable_nodes",
                plugin="composable_nodes::PublisherComponent",
                name="publisher_component",
                parameters=[],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="composable_nodes",
                plugin="composable_nodes::SubscriberComponent",
                name="subscriber_component",
                parameters=[],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
