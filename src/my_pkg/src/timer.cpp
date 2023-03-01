#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class Timer : public rclcpp::Node {
public:
    Timer(std::string node_name)
        : rclcpp::Node(node_name) {
        RCLCPP_INFO(get_logger(), "constructor called");

        timer_base_ = create_wall_timer(500ms, std::bind(&Timer::TimerCallback, this));
    }

    ~Timer() {
        RCLCPP_INFO(get_logger(), "deconstructor called");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_base_;
    void TimerCallback();
};

void Timer::TimerCallback() {
    static size_t counter = 0;
    RCLCPP_INFO(get_logger(), "timer callback %03ld", ++counter);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto simple_node = std::make_shared<Timer>("simple_node");
    rclcpp::spin(simple_node);

    rclcpp::shutdown();
    return 0;
}