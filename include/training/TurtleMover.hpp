#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>                 
#include <rclcpp_action/rclcpp_action.hpp> 
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <training/action/move.hpp>



namespace composition{

class TurtleMover : public rclcpp::Node {

public:
    using Move = training::action::Move;
    using GoalHandleMove =
        rclcpp_action::ServerGoalHandle<Move>;

    explicit TurtleMover(const rclcpp::NodeOptions &options);


private:
    rclcpp_action::Server<Move>::SharedPtr action_server;
    // publisher to publish moving turtle commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

    // subscriber to get moving turt posiiton
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Move::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMove> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

    float curr_x;
    float curr_y;
    float curr_theta;

    float goal_x;
    float goal_y;

    const float TOL = 0.05;
    const float STEP_SCALER = 0.1;

    float distance();

};

} // namespace composition