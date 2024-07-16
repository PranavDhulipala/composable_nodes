#include <memory>
#include <thread>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "composable_nodes/publisher_component.hpp"
#include "composable_nodes/subscriber_component.hpp"

class TestPublisherSubscriber : public ::testing::Test
{
protected:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    std::shared_ptr<composable_nodes::PublisherComponent> publisher_node;
    std::shared_ptr<composable_nodes::SubscriberComponent> subscriber_node;
    std::thread spin_thread;

    void SetUp() override
    {
        rclcpp::init(0, nullptr);

        publisher_node = std::make_shared<composable_nodes::PublisherComponent>(rclcpp::NodeOptions());
        subscriber_node = std::make_shared<composable_nodes::SubscriberComponent>(rclcpp::NodeOptions());

        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_node(publisher_node);
        executor->add_node(subscriber_node);

        spin_thread = std::thread([this]()
                                  { this->executor->spin(); });
    }

    void TearDown() override
    {
        executor->cancel();
        spin_thread.join();

        rclcpp::shutdown();
    }
};

TEST_F(TestPublisherSubscriber, ReceivesMessage)
{
    std::this_thread::sleep_for(std::chrono::seconds(1));

    EXPECT_EQ(subscriber_node->getLastMessage(), "Hello, world! 0");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    if (result == 0) {
        std::cout << "All tests passed" << std::endl;
    }
    return result;
}
