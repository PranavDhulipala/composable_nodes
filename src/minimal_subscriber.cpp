#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "composable_nodes/subscriber_component.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberComponent>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}