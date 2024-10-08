#include "training/TurtleSpawner.hpp"
#include "training/Constants.hpp"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

TurtleSpawner::TurtleSpawner(const rclcpp::NodeOptions &options) : Node("turtle_spawner", options) {
    server = this->create_service<training::srv::Reset>("/spawn_turtle",
        std::bind(&TurtleSpawner::spawn_turtle, this, _1, _2));
    client = this->create_client<turtlesim::srv::Spawn>("/spawn");
}

void TurtleSpawner::spawn_turtle(const std::shared_ptr<training::srv::Reset::Request> request,
    std::shared_ptr<training::srv::Reset::Response> response) {

    (void)request;
    
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    for (const Constants::turtle_info& t : Constants::turtle_bio) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = t.name;
        request->x = t.x_pos;
        request->y = t.y_pos;
        request->theta = t.rad;

        auto result = client->async_send_request(request);
    }

    response->success = true;
}
} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::TurtleSpawner)
