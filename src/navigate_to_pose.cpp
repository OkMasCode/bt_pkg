#include "bt_pkg/navigate_to_pose.hpp"

BT::NodeStatus NavigateToPose::onStart()
{
    // 1. Get the ROS Node from the Blackboard
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }

    // 2. Create the Action Client (if it doesn't exist yet)
    if (!action_client_) {
        action_client_ = rclcpp_action::create_client<Nav2Action>(node, "navigate_to_pose");
    }

    // 3. Wait for the Nav2 Server to be online
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "[NavigateToPose] Action server not available");
        return BT::NodeStatus::FAILURE;
    }

    // 4. Get the Goal from the Input Port
    Nav2Action::Goal goal_msg;
    if (!getInput("goal", goal_msg.pose)) {
        RCLCPP_ERROR(node->get_logger(), "[NavigateToPose] Missing 'goal' input");
        return BT::NodeStatus::FAILURE;
    }

    // Ensure the timestamp is current
    goal_msg.pose.header.stamp = node->now();

    RCLCPP_INFO(node->get_logger(), "[NavigateToPose] Sending Goal: x=%.2f, y=%.2f", 
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    // 5. Send Goal (Asynchronous)
    auto send_goal_options = rclcpp_action::Client<Nav2Action>::SendGoalOptions();
    
    // We request the goal and save the future
    future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPose::onRunning()
{
    // STEP 1: Wait for Goal Acceptance
    // We check if we are still waiting for the server to accept the request
    if (future_goal_handle_.valid()) {
        auto status = future_goal_handle_.wait_for(std::chrono::milliseconds(0));
        
        if (status == std::future_status::ready) {
            goal_handle_ = future_goal_handle_.get();
            future_goal_handle_ = {}; // Clear future so we don't check it again

            if (!goal_handle_) {
                // The server rejected the goal (e.g. goal is outside the map)
                return BT::NodeStatus::FAILURE;
            }
            
            // Goal accepted! Now we start waiting for the Result (arrival)
            future_result_ = action_client_->async_get_result(goal_handle_);
        
        } else {
            return BT::NodeStatus::RUNNING; // Still waiting for acceptance
        }
    }

    // STEP 2: Wait for Arrival (Result)
    if (future_result_.valid()) {
        auto status = future_result_.wait_for(std::chrono::milliseconds(0));
        
        if (status == std::future_status::ready) {
            auto result = future_result_.get();
            
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE; // Canceled, Aborted, or Crashed
            }
        }
    }

    // If we are here, the robot is still moving
    return BT::NodeStatus::RUNNING;
}

void NavigateToPose::onHalted()
{
    // If the behavior tree stops this node (e.g. timeout), cancel the goal
    if (action_client_ && goal_handle_) {
        action_client_->async_cancel_goal(goal_handle_);
    }
}