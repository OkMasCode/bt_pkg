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

class NavigateToPose : public BT::StatefulActionNode
{
public:
    NavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // INPUT: The target coordinate (from "CallCheckCandidates" or "SelectCluster")
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
        };
    }

    // Standard V4 Stateful Methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    Nav2Client::SharedPtr action_client_;
    
    // Future 1: Waiting for the server to say "I accept this goal"
    std::shared_future<GoalHandleNav::SharedPtr> future_goal_handle_;
    
    // Future 2: Waiting for the robot to say "I arrived"
    std::shared_future<GoalHandleNav::WrappedResult> future_result_;
    
    // The active goal handle (needed to cancel if we stop)
    GoalHandleNav::SharedPtr goal_handle_;
};