#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>
#include <memory>
#include <string>
#include <vector>


namespace composition {

class TurtleClearer : public rclcpp::Node {
public:
    TurtleClearer(const rclcpp::NodeOptions &options);
private:
    void clear_turtle();
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
    std::vector<std::string> turtle_names = {"turtle1", "moving_turtle",
                                           "stationary_turtle"};
};

} // namespace composition