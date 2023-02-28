#include <rclcpp/rclcpp.hpp>
#include <my_pkg/msg/str.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber(std::string node_name)
        : rclcpp::Node(node_name) {
        RCLCPP_INFO(get_logger(), "constructor called");

        // subscriber
        subscription_ = create_subscription<my_pkg::msg::Str>("str", 10, std::bind(&Subscriber::StrSubCallback, this, _1));
    }

    ~Subscriber() {
        RCLCPP_INFO(get_logger(), "deconstructor called");
    }

private:
    rclcpp::Subscription<my_pkg::msg::Str>::SharedPtr subscription_;
    void StrSubCallback(my_pkg::msg::Str::SharedPtr msg);
};

void Subscriber::StrSubCallback(my_pkg::msg::Str::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "subscribe message : %s", msg->data.c_str());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto subscriber = std::make_shared<Subscriber>("subscriber");
    rclcpp::spin(subscriber);

    rclcpp::shutdown();
    return 0;
}