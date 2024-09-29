#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "training/TurtleSpawner.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<composition::TurtleSpawner>(rclcpp::NodeOptions()));
    
    rclcpp::shutdown();
    return 0;
}
