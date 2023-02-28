#include <rclcpp/rclcpp.hpp>
#include <my_pkg/msg/str.hpp>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
public:
    Publisher(std::string node_name)
        : rclcpp::Node(node_name) {
        RCLCPP_INFO(get_logger(), "constructor called");

        // timer
        timer_base_ = create_wall_timer(500ms, std::bind(&Publisher::TimerCallback, this));

        // publisher
        publisher_ = create_publisher<my_pkg::msg::Str>("str", 10);
    }

    ~Publisher() {
        RCLCPP_INFO(get_logger(), "deconstructor called");
    }

private:
    // timer
    rclcpp::TimerBase::SharedPtr timer_base_;
    void TimerCallback();

    // publisher
    rclcpp::Publisher<my_pkg::msg::Str>::SharedPtr publisher_;
};

void Publisher::TimerCallback() {
    static size_t counter = 0;
    my_pkg::msg::Str message;
    message.data = "msg " + std::to_string(++counter);
    RCLCPP_INFO(get_logger(), "publish message : %s", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto publisher = std::make_shared<Publisher>("publisher");
    rclcpp::spin(publisher);

    rclcpp::shutdown();
    return 0;
}