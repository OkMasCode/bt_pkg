#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

// Condition node that checks whether the current target corresponds to an object.
class CheckIsObject : public BT::ConditionNode
{
public:
    CheckIsObject(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // True when the selected target is an object goal.
            BT::InputPort<bool>("is_object")
        };
    }

    BT::NodeStatus tick() override
    {
        // Missing input is treated as a failed condition.
        bool is_object = false;
        if (!getInput("is_object", is_object)) {
            return BT::NodeStatus::FAILURE;
        }
        // Succeeds only when target is an object.
        return is_object ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
