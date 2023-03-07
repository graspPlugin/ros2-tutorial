#include <rclcpp/rclcpp.hpp>
#include <my_pkg/srv/sample_srv.hpp>

using namespace std::placeholders;

class Server : public rclcpp::Node {
public:
    Server(std::string node_name)
        : rclcpp::Node(node_name) {
        RCLCPP_INFO(get_logger(), "Constructor called");

        service_ = create_service<my_pkg::srv::SampleSrv>("sample_srv", std::bind(&Server::SampleSrvCallback, this, _1, _2));
    }
    ~Server() {
        RCLCPP_INFO(get_logger(), "Deconstructor called");
    }

private:
    rclcpp::Service<my_pkg::srv::SampleSrv>::SharedPtr service_;
    void SampleSrvCallback(const std::shared_ptr<my_pkg::srv::SampleSrv::Request> request,
        std::shared_ptr<my_pkg::srv::SampleSrv::Response> response);
};

void Server::SampleSrvCallback(const std::shared_ptr<my_pkg::srv::SampleSrv::Request> request,
    std::shared_ptr<my_pkg::srv::SampleSrv::Response> response) {
    RCLCPP_INFO(get_logger(), "Receive request");
    RCLCPP_INFO(get_logger(), "  -> f32_1 = %f", request->f32_1);
    RCLCPP_INFO(get_logger(), "  -> f32_2 = %f", request->f32_2);

    response->f32 = request->f32_1 + request->f32_2;
    RCLCPP_INFO(get_logger(), "Send response");
    RCLCPP_INFO(get_logger(), "  -> f32 = %f", response->f32);
    RCLCPP_INFO(get_logger(), "---");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto server = std::make_shared<Server>("server");
    rclcpp::spin(server);

    rclcpp::shutdown();
    return 0;
}