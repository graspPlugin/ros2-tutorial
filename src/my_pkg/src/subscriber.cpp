#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Subscriber : public rclcpp::Node {
public:
    Subscriber(std::string node_name)
        : rclcpp::Node(node_name) {
        RCLCPP_INFO(get_logger(), "constructor called");

        subscription_ = create_subscription<std_msgs::msg::String>("my_chatter", 10, std::bind(&Subscriber::SubMsgCallback, this, std::placeholders::_1));
    }

    ~Subscriber() {
        RCLCPP_INFO(get_logger(), "deconstructor called");
    }

private:
    // subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void SubMsgCallback(std_msgs::msg::String::SharedPtr msg);
};

void Subscriber::SubMsgCallback(std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "I heard: [%s]", msg->data.c_str());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto subscriber = std::make_shared<Subscriber>("subscriber");
    rclcpp::spin(subscriber);

    rclcpp::shutdown();
    return 0;
}