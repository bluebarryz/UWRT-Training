#include <training/TurtleMover.hpp>
#include <cmath>
#include <cassert>
#include <thread>
#include <chrono>




using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

TurtleMover::TurtleMover(const rclcpp::NodeOptions &options) : 
    Node("moving_turtle_action_server", options), curr_x{-1}, curr_y{-1}, goal_x{-2}, goal_y{-2} {

    this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/moving_turtle/cmd_vel", 10);

    auto subscriber_callback =
        [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
            this->curr_x = msg->x;
            this->curr_y = msg->y;
            this->curr_theta = msg->theta;
        };

    this->subscriber = this->create_subscription<turtlesim::msg::Pose>(
        "/moving_turtle/pose", 10, subscriber_callback);

    this->action_server =
        rclcpp_action::create_server<Move>(
            this, "move_turtle_to_waypoint",
            std::bind(&TurtleMover::handle_goal, this, _1, _2),
            std::bind(&TurtleMover::handle_cancel, this, _1),
            std::bind(&TurtleMover::handle_accepted, this, _1)

        );
}

rclcpp_action::GoalResponse TurtleMover::handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Move::Goal> goal) {

    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Goal Received");
    RCLCPP_INFO(this->get_logger(), "linear X:%f Y:%f", goal->x_pos, goal->y_pos);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse TurtleMover::handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleMover::handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TurtleMover::execute, this, _1), goal_handle}.detach();
}

float TurtleMover::distance() {
    assert(curr_x >=0 && curr_y >= 0 && goal_x >= 0 && goal_y >= 0);

    float x_dist = goal_x - curr_x;
    float y_dist = goal_y - curr_y;

    return std::sqrt(std::pow(x_dist, 2) + std::pow(y_dist, 2));
}

void TurtleMover::execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    rclcpp::Time start_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    goal_x = goal->x_pos;
    goal_y = goal->y_pos;
    auto feedback = std::make_shared<Move::Feedback>();
    auto result = std::make_shared<Move::Result>();


    while(rclcpp::ok() && distance() > TOL) {
        
        if (goal_handle->is_canceling()) {
            rclcpp::Time curr_time = this->now();
            rclcpp::Duration time = curr_time - start_time;
            long int duration{time.nanoseconds()};
            result->duration = duration;

            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            return;
        }

        // Publish move msg to cmd_vel topic
        auto msg = std::make_unique<geometry_msgs::msg::Twist>();
        float x_step= (goal_x - curr_x) * STEP_SCALER;
        float y_step = (goal_y - curr_y) * STEP_SCALER;
        msg->linear.x = x_step;
        msg->linear.y = y_step;
        msg->linear.z = 0;
        msg->angular.x = 0;
        msg->angular.y = 0;
        msg->angular.z = 0;
        publisher->publish(std::move(msg));
        
        feedback->dist_remaining = distance();
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
        rclcpp::Time curr_time = this->now();
        rclcpp::Duration time = curr_time - start_time;
        long int duration{time.nanoseconds()};
        result->duration = duration;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::TurtleMover)