#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class ReadJson : public BT::SyncActionNode
{
public:
    ReadJson(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // Input: Path to the file
            BT::InputPort<std::string>("file_path"),
            
            // Outputs: Data for the rest of the tree
            BT::OutputPort<std::vector<std::string>>("candidats_ids"),
            BT::OutputPort<std::vector<std::string>>("prompt"), 
            BT::OutputPort<geometry_msgs::msg::Pose>("cluster_id")
        };
    }

    // Since it is SyncActionNode, we just override tick(), not onStart/onRunning
    BT::NodeStatus tick() override;
};