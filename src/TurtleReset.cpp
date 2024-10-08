#include "training/TurtleReset.hpp"
#include "training/Constants.hpp"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

TurtleReset::TurtleReset(const rclcpp::NodeOptions &options) : Node("turtle_clearer", options) {
    server = this->create_service<training::srv::Reset>("/reset_moving_turtle",
        std::bind(&TurtleReset::reset_turtle, this, _1, _2));
    client = this->create_client<turtlesim::srv::TeleportAbsolute>("/moving_turtle/teleport_absolute");

}

void TurtleReset::reset_turtle(const std::shared_ptr<training::srv::Reset::Request> request,
    std::shared_ptr<training::srv::Reset::Response> response) {

    (void)request;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto reset_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    const Constants::turtle_info& moving_turtle_bio = Constants::turtle_bio[1]; 
    reset_request->x = moving_turtle_bio.x_pos;
    reset_request->y = moving_turtle_bio.y_pos;
    reset_request->theta = 0;

    auto result = client->async_send_request(reset_request);

    response->success = true;

}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::TurtleReset)
