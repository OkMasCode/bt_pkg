#pragma once

#include "behaviortree_cpp/action_node.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#include "bt_pkg/logic_type.hpp"
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
            
            // Command-level fields consumed by SelectGoal.
            BT::OutputPort<std::string>("goal_class"),
            BT::OutputPort<int>("cluster"),
            // High-level task action (e.g., bring_back_object).
            BT::OutputPort<std::string>("action"),
            BT::OutputPort<LogicType>("logic")
        };
    }

    // Executes parsing synchronously and writes all outputs in one tick.
    BT::NodeStatus tick() override;
};