// #include <rclcpp/rclcpp.hpp>
// #include <my_pkg/srv/sample_srv.hpp>

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);

//     if (argc != 3) {
//         RCLCPP_INFO(node->get_logger(), "usage: sample_srv f32_data1 f32_data2");
//         return 1;
//     }
//     float f32_data1 = atof(argv[1]);
//     float f32_data2 = atof(argv[2]);

//     // create client node
//     auto client_node = rclcpp::Node::make_shared("client");

//     // create client
//     auto client = client_node->create_client<my_pkg::srv::SampleSrv>("sample_srv");
//     my_pkg::srv::SampleSrv::Request::SharedPtr request;
//     request->f32_data1 = f32_data1;
//     request->f32_data2 = f32_data2;

//     // send request
//     auto result = client->async_send_request(request);
//     // receive response
//     if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_INFO(client_node->get_logger(), "Receive response");
//         RCLCPP_INFO(client_node->get_logger(), "--- f32_data = %f", result.get()->f32_data);
//     }

//     // shutdown
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <my_pkg/srv/sample_srv.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("client"), "usage: client X Y");
        return 1;
    }

    auto node = rclcpp::Node::make_shared("client");
    auto client = node->create_client<my_pkg::srv::SampleSrv>("sample_srv");

    auto request = std::make_shared<my_pkg::srv::SampleSrv::Request>();
    request->f32_data1 = atof(argv[1]);
    request->f32_data2 = atof(argv[2]);

    while (!client->wait_for_service(500ms)) {
        RCLCPP_INFO(node->get_logger(), "Service not available");
        if (!rclcpp::ok()) {
            return 0;
        }
    }

    // send request
    RCLCPP_INFO(node->get_logger(), "Send request");
    RCLCPP_INFO(node->get_logger(), "  -> f32_data1 = %f", request->f32_data1);
    RCLCPP_INFO(node->get_logger(), "  -> f32_data2 = %f", request->f32_data2);
    auto result = client->async_send_request(request);

    // wait for response
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Receive response");
        RCLCPP_INFO(node->get_logger(), "  -> f32_data = %f", result.get()->f32_data);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}