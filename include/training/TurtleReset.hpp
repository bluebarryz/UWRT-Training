#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <training/srv/reset.hpp>
#include <memory>


namespace composition {

class TurtleReset : public rclcpp::Node {
public:
    explicit TurtleReset(const rclcpp::NodeOptions &options);
private:
    void reset_turtle(const std::shared_ptr<training::srv::Reset::Request> request,
        std::shared_ptr<training::srv::Reset::Response> response);

    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client;
    rclcpp::Service<training::srv::Reset>::SharedPtr server;
};

} // namespace composition