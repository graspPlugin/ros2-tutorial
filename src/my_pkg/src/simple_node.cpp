#include <rclcpp/rclcpp.hpp>

class SimpleNode : public rclcpp::Node {
public:
  SimpleNode(std::string node_name) : rclcpp::Node(node_name) {
    RCLCPP_INFO(get_logger(), "constructor called");
  }

  ~SimpleNode(){
    RCLCPP_INFO(get_logger(), "deconstructor called");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto simple_node = std::make_shared<SimpleNode>("simple_node");
  rclcpp::spin(simple_node);

  rclcpp::shutdown();
  return 0;
}