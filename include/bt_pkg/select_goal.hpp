#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

// Synchronous BT action that selects a navigation target from perception candidates.
class SelectGoal : public BT::SyncActionNode
{
public:
    SelectGoal(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
    {}

    // Ports consumed/produced by this node.
    static BT::PortsList providedPorts()
    {
        return {
            // Candidate object IDs associated with each detected goal pose.
            BT::InputPort<std::vector<std::string>>("candidates_ids"),
            // Similarity score for each candidate (same indexing as goal_poses).
            BT::InputPort<std::vector<double>>("similarity_scores"),
            // Candidate object poses returned by perception.
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goal_poses"),
            // Fallback exploration pose when no confident object match is found.
            BT::InputPort<geometry_msgs::msg::PoseStamped>("cluster_centroid"),
            // Minimum score required to accept a candidate object pose.
            BT::InputPort<double>("similarity_threshold"),
            
            // Selected navigation target (object pose or cluster centroid).
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            // True when target_pose is an object goal, false when it is the centroid.
            BT::OutputPort<bool>("is_object_goal")
        };
    }

    // Chooses the best target and writes outputs to the blackboard.
    BT::NodeStatus tick() override;
};