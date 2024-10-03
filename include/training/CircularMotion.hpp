#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>



namespace composition {

class CircularMotion : public rclcpp::Node {
public:
    explicit CircularMotion(const rclcpp::NodeOptions &options);
private:
    void motion_callback();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

} // namespace composition