#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "composable_nodes/publisher_component.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherComponent>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}