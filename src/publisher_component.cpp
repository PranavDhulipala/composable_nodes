
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "composable_nodes/publisher_component.hpp"

namespace composable_nodes
{
    PublisherComponent::PublisherComponent(const rclcpp::NodeOptions &options)
        : Node("publisher_component", options), message_count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PublisherComponent::publishMessage, this));
    }

    void PublisherComponent::publishMessage()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(message_count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(composable_nodes::PublisherComponent)