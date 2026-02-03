#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

class CheckIsObject : public BT::ConditionNode
{
public:
    CheckIsObject(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("is_object")
        };
    }

    BT::NodeStatus tick() override
    {
        bool is_object = false;
        if (!getInput("is_object", is_object)) {
            return BT::NodeStatus::FAILURE;
        }
        return is_object ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
