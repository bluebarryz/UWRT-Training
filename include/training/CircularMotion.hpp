#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>
#include <training/srv/reset.hpp>

namespace composition {

class CircularMotion : public rclcpp::Node {
public:
    explicit CircularMotion(const rclcpp::NodeOptions &options);
private:
    void motion_callback();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;

    void service_callback(const std::shared_ptr<training::srv::Reset::Request> request,
        std::shared_ptr<training::srv::Reset::Response> response);
    
    rclcpp::Service<training::srv::Reset>::SharedPtr server;
};

} // namespace composition