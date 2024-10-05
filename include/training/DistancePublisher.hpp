#include <rclcpp/rclcpp.hpp>
#include <training/msg/turtle_dist.hpp>
#include "turtlesim/msg/pose.hpp"



namespace composition {

class DistancePublisher : public rclcpp::Node {
public:
    explicit DistancePublisher(const rclcpp::NodeOptions &options);
private:
    rclcpp::Publisher<training::msg::TurtleDist>::SharedPtr publisher;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr stationary_turt_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_turt_sub;

    float x_stationary_turt;
    float y_stationary_turt;

    float x_moving_turt;
    float y_moving_turt;

    rclcpp::TimerBase::SharedPtr timer;

    void compute_and_publish_distance();
};

} // namespace composition