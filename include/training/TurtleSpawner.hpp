#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <vector>
#include <string>

namespace composition {

class TurtleSpawner : public rclcpp::Node {
public:
	TurtleSpawner(const rclcpp::NodeOptions &options);
private:
	void spawn_turtle();
	rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
	struct turtle_info {
		std::string name;
		float x_pos;
		float y_pos;
		float rad;
	};

	std::vector<turtle_info> turtle_bio{{"stationary_turtle", 5, 5, 0},
                                      {"moving_turtle", 25, 10, 0}};
};

} // namespace composition