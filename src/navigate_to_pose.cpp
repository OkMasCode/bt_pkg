#include "bt_pkg/navigate_to_pose.hpp"

BT::NodeStatus NavigateToPose::onStart()
{
    // Retrieve ROS node from blackboard for client creation and logging.
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }

    // Lazily create action client once.
    if (!action_client_) {
        action_client_ = rclcpp_action::create_client<Nav2Action>(node, "navigate_to_pose");
    }

    // Ensure Nav2 action server is available.
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "[NavigateToPose] Action server not available");
        return BT::NodeStatus::FAILURE;
    }

    // Read target goal pose from input port.
    Nav2Action::Goal goal_msg;
    if (!getInput("goal", goal_msg.pose)) {
        RCLCPP_ERROR(node->get_logger(), "[NavigateToPose] Missing 'goal' input");
        return BT::NodeStatus::FAILURE;
    }

    // Refresh timestamp before sending.
    goal_msg.pose.header.stamp = node->now();

    RCLCPP_INFO(node->get_logger(), "[NavigateToPose] Sending Goal: x=%.2f, y=%.2f", 
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    // Send goal asynchronously; completion is handled in onRunning().
    auto send_goal_options = rclcpp_action::Client<Nav2Action>::SendGoalOptions();
    
    // Store future for non-blocking acceptance check.
    future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPose::onRunning()
{
    // Phase 1: poll for goal acceptance.
    if (future_goal_handle_.valid()) {
        auto status = future_goal_handle_.wait_for(std::chrono::milliseconds(0));
        
        if (status == std::future_status::ready) {
            goal_handle_ = future_goal_handle_.get();
            future_goal_handle_ = {}; // Clear future so we don't check it again

            if (!goal_handle_) {
                // Goal rejected by server.
                return BT::NodeStatus::FAILURE;
            }
            
            // Goal accepted: start polling for final result.
            future_result_ = action_client_->async_get_result(goal_handle_);
        
        } else {
            return BT::NodeStatus::RUNNING; // Still waiting for acceptance
        }
    }

    // Phase 2: poll for navigation result.
    if (future_result_.valid()) {
        auto status = future_result_.wait_for(std::chrono::milliseconds(0));
        
        if (status == std::future_status::ready) {
            auto result = future_result_.get();
            
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE; // Canceled/aborted/failed
            }
        }
    }

    // Navigation still in progress.
    return BT::NodeStatus::RUNNING;
}

void NavigateToPose::onHalted()
{
    // If halted by the tree, request cancellation of the active goal.
    if (action_client_ && goal_handle_) {
        action_client_->async_cancel_goal(goal_handle_);
    }
}