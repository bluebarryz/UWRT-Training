#include "training/CircularMotion.hpp"

using namespace std::chrono_literals;

namespace composition {

CircularMotion::CircularMotion(const rclcpp::NodeOptions &options) : Node("circular_motion_publisher", options) {
	publisher = this->create_publisher<geometry_msgs::msg::Twist>(
	"/turtle1/cmd_vel", 10);

	timer = this->create_wall_timer(100ms, std::bind(&CircularMotion::motion_callback, this));
}

void CircularMotion::motion_callback() {
	auto message = geometry_msgs::msg::Twist();
	message.linear.x = 2;

	message.angular.z = 3;

	publisher->publish(std::move(message));
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::CircularMotion)