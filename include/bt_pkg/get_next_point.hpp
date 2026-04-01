#pragma once

// FIXED: Using the correct v4 include path
#include "behaviortree_cpp/action_node.h" 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <string>

class GetNextPoint : public BT::SyncActionNode
{
public:
    GetNextPoint(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("cluster_centroid"),
            BT::InputPort<std::vector<double>>("cluster_dimensions"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("next_point")
        };
    }

    BT::NodeStatus tick() override;
};