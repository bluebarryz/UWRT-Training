#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <vector>
#include <string>
#include <training/srv/reset.hpp>

namespace composition {

class TurtleSpawner : public rclcpp::Node {
public:
	explicit TurtleSpawner(const rclcpp::NodeOptions &options);
private:
	void spawn_turtle(const std::shared_ptr<training::srv::Reset::Request> request,
        std::shared_ptr<training::srv::Reset::Response> response);

	rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
    rclcpp::Service<training::srv::Reset>::SharedPtr server;
};

} // namespace composition