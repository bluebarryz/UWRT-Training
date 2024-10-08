#include "training/CircularMotion.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

CircularMotion::CircularMotion(const rclcpp::NodeOptions &options) : Node("circular_motion_publisher", options) {
	publisher = this->create_publisher<geometry_msgs::msg::Twist>(
	"/turtle1/cmd_vel", 10);

	server = this->create_service<training::srv::Reset>("/circular_motion",
		std::bind(&CircularMotion::service_callback, this, _1, _2));
}

void CircularMotion::motion_callback() {
	auto message = geometry_msgs::msg::Twist();
	message.linear.x = 2;
	message.angular.z = 3;

	publisher->publish(std::move(message));
}

void CircularMotion::service_callback(const std::shared_ptr<training::srv::Reset::Request> request,
    std::shared_ptr<training::srv::Reset::Response> response) {
	
	(void)request;

	timer = this->create_wall_timer(100ms, std::bind(&CircularMotion::motion_callback, this));
	response->success = true;
}


} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::CircularMotion)