#ifndef COMPOSABLE_NODES__SUBSCRIBER_NODE_H
#define COMPOSABLE_NODES__SUBSCRIBER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "composable_nodes/visibility.h"

/**
 * @brief SubscriberComponent class that subscribes to a topic and logs received messages
 */
class SubscriberComponent : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Subscriber Node object
     *
     * @param options The options for this node instance
     */
    explicit COMPOSABLE_NODES_PUBLIC SubscriberComponent(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Get the last message received by the subscriber
     * 
     * @return std::string The last message received
     */
    std::string getLastMessage() const { return last_message_; }

private:
    /**
     * @brief Callback function for topic messages
     *
     * @param msg The message received from the topic
     */
    void topicCallback(const std_msgs::msg::String::SharedPtr msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; // The subscription object
    mutable std::string last_message_;                                    // The last message received
};

#endif // COMPOSABLE_NODES__SUBSCRIBER_NODE_H