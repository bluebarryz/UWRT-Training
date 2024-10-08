#include "training/TurtleClearer.hpp"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

TurtleClearer::TurtleClearer(const rclcpp::NodeOptions &options) : Node("turtle_clearer", options) {
    server = this->create_service<training::srv::Reset>("/clear_turtle",
        std::bind(&TurtleClearer::clear_turtle, this, _1, _2));
    client = this->create_client<turtlesim::srv::Kill>("/kill");
}

void TurtleClearer::clear_turtle(const std::shared_ptr<training::srv::Reset::Request> request,
    std::shared_ptr<training::srv::Reset::Response> response) {
    
    (void)request;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    for (std::string &name : turtle_names) {
        auto req = std::make_shared<turtlesim::srv::Kill::Request>();
        req->name = name;
        RCLCPP_INFO(this->get_logger(), "%s", req->name.c_str());
        auto result = client->async_send_request(req);
        // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        //     RCLCPP_INFO(this->get_logger(), "Successfully cleared %s", name.c_str());
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to clear %s", name.c_str());
        // }
    }

    response->success = true;
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::TurtleClearer)
