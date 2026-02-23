#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Short definitions for easier reading
using Nav2Action = nav2_msgs::action::NavigateToPose;
using Nav2Client = rclcpp_action::Client<Nav2Action>;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<Nav2Action>;

// Stateful BT node that sends a Nav2 goal and tracks completion asynchronously.
class NavigateToPose : public BT::StatefulActionNode
{
public:
    NavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // Target pose to send to Nav2.
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
        };
    }

    // BT lifecycle callbacks for stateful execution.
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // Nav2 action client instance.
    Nav2Client::SharedPtr action_client_;
    
    // Future for goal-acceptance response.
    std::shared_future<GoalHandleNav::SharedPtr> future_goal_handle_;
    
    // Future for final navigation result.
    std::shared_future<GoalHandleNav::WrappedResult> future_result_;
    
    // Active goal handle, used for cancellation on halt.
    GoalHandleNav::SharedPtr goal_handle_;
};