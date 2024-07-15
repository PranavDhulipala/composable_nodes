#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "composable_nodes/subscriber_component.hpp"

SubscriberComponent::SubscriberComponent(const rclcpp::NodeOptions &options)
    : Node("subscriber_component", options)
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10,
        std::bind(&SubscriberComponent::topicCallback, this, std::placeholders::_1));
}

void SubscriberComponent::topicCallback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    last_message_ = msg->data;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SubscriberComponent)