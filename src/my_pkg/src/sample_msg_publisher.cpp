#include <rclcpp/rclcpp.hpp>
#include <my_pkg/msg/sample_msg.hpp>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
public:
    Publisher(std::string node_name)
        : rclcpp::Node(node_name) {
        RCLCPP_INFO(get_logger(), "constructor called");

        timer_base_ = create_wall_timer(500ms, std::bind(&Publisher::TimerCallback, this));
        publisher_ = create_publisher<my_pkg::msg::SampleMsg>("sample_msg", 10);
    }

    ~Publisher() {
        RCLCPP_INFO(get_logger(), "deconstructor called");
    }

private:
    // timer
    rclcpp::TimerBase::SharedPtr timer_base_;
    void TimerCallback();
    // publisher
    rclcpp::Publisher<my_pkg::msg::SampleMsg>::SharedPtr publisher_;
};

void Publisher::TimerCallback() {
    static size_t counter = 0;
    my_pkg::msg::SampleMsg msg;
    msg.str_1 = "Hello World1: " + std::to_string(counter);
    msg.str_2 = "Hello World2: " + std::to_string(counter);
    msg.i = counter;
    publisher_->publish(msg);

    RCLCPP_INFO(get_logger(), "Publishing str_1 :'%s'", msg.str_1.c_str());
    RCLCPP_INFO(get_logger(), "Publishing str_2 :'%s'", msg.str_2.c_str());
    RCLCPP_INFO(get_logger(), "Publishing     i :'%d'", msg.i);
    RCLCPP_INFO(get_logger(), "---");
    counter++;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto publisher = std::make_shared<Publisher>("publisher");
    rclcpp::spin(publisher);

    rclcpp::shutdown();
    return 0;
}