#pragma once

#include <mutex>
#include <string>

// Include the necessary message types
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp" // Added for the boolean flag
#include "rclcpp/rclcpp.hpp"

class CheckGoalSeen : public BT::ConditionNode
{
public:
    CheckGoalSeen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal_pose_topic"),
            BT::InputPort<std::string>("goal_flag_topic"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
        };
    }

    BT::NodeStatus tick() override;

private:
    void ensureSubscription(rclcpp::Node::SharedPtr& node);
    void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onGoalFlag(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    
    // Subscribers for both topics
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_flag_sub_;
    
    // Tracking the names of the subscribed topics to prevent redundant subscriptions
    std::string subscribed_pose_topic_;
    std::string subscribed_flag_topic_;

    std::mutex mutex_;
    
    // Data storage and state flags
    geometry_msgs::msg::PoseStamped latest_goal_pose_;
    bool has_goal_pose_{false};
    bool is_goal_seen_{false}; // Tracks the actual boolean flag value directly
};