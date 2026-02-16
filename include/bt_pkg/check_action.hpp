#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

class CheckAction : public BT::ConditionNode
{
public:
    CheckAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("action")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string action;
        if (!getInput("action", action)) {
            return BT::NodeStatus::FAILURE;
        }
        return action == "bring_back_object" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
