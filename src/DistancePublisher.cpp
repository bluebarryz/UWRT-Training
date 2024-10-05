#include <training/DistancePublisher.hpp>

using namespace std::chrono_literals;

namespace composition {

DistancePublisher::DistancePublisher(const rclcpp::NodeOptions &options) : Node("circular_motion_publisher", options) {
	publisher = this->create_publisher<training::msg::TurtleDist>(
	"/distance", 10);

	stationary_turt_sub = this->create_subscription<turtlesim::msg::Pose>(
		"/stationary_turtle/pose", 10,
		[this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
			this->x_stationary_turt = msg->x;
			this->y_stationary_turt = msg->y;
		}
	);

	moving_turt_sub = this->create_subscription<turtlesim::msg::Pose>(
		"/moving_turtle/pose", 10,
		[this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
			this->x_moving_turt = msg->x;
			this->y_moving_turt = msg->y;
		}
	);

	timer = this->create_wall_timer(3s, std::bind(&DistancePublisher::compute_and_publish_distance, this));
}

void DistancePublisher::compute_and_publish_distance() {
	// compute absolute difference in coordinates
	double position_x{abs(this->x_stationary_turt - this->x_moving_turt)};
	double position_y{abs(this->y_stationary_turt - this->y_moving_turt)};

	// create message to publish
	auto msg = std::make_unique<training::msg::TurtleDist>();
	msg->x_pos = position_x;
	msg->y_pos = position_y;
	// compute distance using trig
	msg->distance = sqrt((position_x * position_x) + (position_y * position_y));

	// publish message
	this->publisher->publish(
		std::move(msg));
}


} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::DistancePublisher)