#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
// #include <my_pkg/msg/sample_msg.hpp>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
public:
    Publisher(std::string node_name)
        : rclcpp::Node(node_name) {
        RCLCPP_INFO(get_logger(), "constructor called");

        timer_base_ = create_wall_timer(500ms, std::bind(&Publisher::TimerCallback, this));
        publisher_ = create_publisher<std_msgs::msg::String>("my_chatter", 10);
    }

    ~Publisher() {
        RCLCPP_INFO(get_logger(), "deconstructor called");
    }

private:
    // timer
    rclcpp::TimerBase::SharedPtr timer_base_;
    void TimerCallback();
    // publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

void Publisher::TimerCallback() {
    static size_t counter = 0;
    std_msgs::msg::String msg;
    msg.data = "Hello World: " + std::to_string(++counter);
    publisher_->publish(msg);

    RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto publisher = std::make_shared<Publisher>("publisher");
    rclcpp::spin(publisher);

    rclcpp::shutdown();
    return 0;
}