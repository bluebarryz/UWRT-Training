#include "training/TurtleSpawner.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace composition {

TurtleSpawner::TurtleSpawner(const rclcpp::NodeOptions &options) : Node("turtle_spawner", options) {
    client = this->create_client<turtlesim::srv::Spawn>("/spawn");

    spawn_turtle();
}

void TurtleSpawner::spawn_turtle() {
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    for (const turtle_info& t : turtle_bio) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = t.name;
        request->x = t.x_pos;
        request->y = t.y_pos;
        request->theta = t.rad;

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Successfully spawned %s", t.name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn %s", t.name.c_str());
        }
    }
}
} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::TurtleSpawner)
