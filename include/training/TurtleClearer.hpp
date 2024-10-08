#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>
#include <memory>
#include <string>
#include <vector>
#include <training/srv/reset.hpp>

namespace composition {

class TurtleClearer : public rclcpp::Node {
public:
    explicit TurtleClearer(const rclcpp::NodeOptions &options);
private:
    void clear_turtle(const std::shared_ptr<training::srv::Reset::Request> request,
        std::shared_ptr<training::srv::Reset::Response> response);

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
    rclcpp::Service<training::srv::Reset>::SharedPtr server;

    std::vector<std::string> turtle_names = {"turtle1", "moving_turtle",
                                           "stationary_turtle"};
};

} // namespace composition