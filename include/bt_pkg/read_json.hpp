#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Synchronous BT action that parses a command JSON file and publishes fields to blackboard ports.
class ReadJson : public BT::SyncActionNode
{
public:
    ReadJson(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // Absolute/relative path to the JSON command file.
            BT::InputPort<std::string>("file_path"),
            
            // Candidate objects and scores extracted from goal_objects.
            BT::OutputPort<std::vector<std::string>>("candidates_ids"),
            BT::OutputPort<std::vector<double>>("similarity_scores"),
            BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goal_poses"),
            // CLIP prompts to evaluate.
            BT::OutputPort<std::vector<std::string>>("prompt"), 
            // Selected cluster metadata.
            BT::OutputPort<int>("cluster"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("cluster_centroid"),
            BT::OutputPort<std::string>("cluster_dimensions"),
            // High-level task action (e.g., bring_back_object).
            BT::OutputPort<std::string>("action")
        };
    }

    // Executes parsing synchronously and writes all outputs in one tick.
    BT::NodeStatus tick() override;
};