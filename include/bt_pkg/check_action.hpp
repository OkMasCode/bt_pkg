#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

// Condition node that succeeds only for the bring-back-object high-level action.
class CheckAction : public BT::ConditionNode
{
public:
    CheckAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // Action string to evaluate.
        return {
            BT::InputPort<std::string>("action")
        };
    }

    BT::NodeStatus tick() override
    {
        // If the action is missing, fail the condition to keep tree behavior explicit.
        std::string action;
        if (!getInput("action", action)) {
            return BT::NodeStatus::FAILURE;
        }
        // Pass only when current action is bring_back_object.
        return action == "bring_back_object" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
