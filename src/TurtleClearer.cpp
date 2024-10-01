#include "training/TurtleClearer.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace composition {

TurtleClearer::TurtleClearer(const rclcpp::NodeOptions &options) : Node("turtle_clearer", options) {
    client = this->create_client<turtlesim::srv::Kill>("/clear");

    clear_turtle();
}

void TurtleClearer::clear_turtle() {
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    for (std::string &name : turtle_names) {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Successfully cleared turtle");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to clear turtle");
        }
    }
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::TurtleClearer)
