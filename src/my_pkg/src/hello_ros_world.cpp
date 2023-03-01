#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_ros_world");
    RCLCPP_INFO(node->get_logger(), "hello ros world!!");

    rclcpp::shutdown();

    return 0;
}