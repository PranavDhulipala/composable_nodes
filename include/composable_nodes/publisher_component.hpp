#ifndef COMPOSABLE_NODES__PUBLISHER_NODE_H
#define COMPOSABLE_NODES__PUBLISHER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "composable_nodes/visibility.h"

/**
 * @class PublisherComponent
 * @brief A ROS 2 node that publishes messages of type std_msgs::msg::String
 */
class PublisherComponent : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Publisher Component object
   *
   * @param options The options for this node instance, allowing for more flexible configuration
   */
  explicit COMPOSABLE_NODES_PUBLIC PublisherComponent(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /**
   * @brief Publishes a message to a topic.
   */
  void publishMessage();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // The publisher object
  rclcpp::TimerBase::SharedPtr timer_;                            // Timer to schedule message publishing
  size_t message_count_;                                          // Counter for messages
};

#endif // COMPOSABLE_NODES__PUBLISHER_NODE_H