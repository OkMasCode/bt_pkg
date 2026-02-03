#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

class CallCheckCandidates : public BT::SyncActionNode
{
public:
    CallCheckCandidates(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
    {}

    // Define the inputs (from JSON) and outputs (to Navigation)
    static BT::PortsList providedPorts()
    {
        return {
            // INPUTS: defined in ReadJson and passed here via XML
            BT::InputPort<std::vector<std::string>>("candidates_ids"),
            BT::InputPort<std::vector<double>>("similarity_scores"),
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goal_poses"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("cluster_centroid"),
            BT::InputPort<double>("similarity_threshold"),
            
            // OUTPUT: The target for NavigateToPose
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            BT::OutputPort<bool>("is_object_goal")
        };
    }

    BT::NodeStatus tick() override;
};