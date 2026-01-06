#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

class RunLLM : public BT::StatefulActionNode
{
public:
    RunLLM(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // Define your input and output ports here
            BT::InputPort<std::string>("file_path"),

            BT::OutputPort<std::vector<std::string>>("candidate_ids"),
            BT::OutputPort<std::vector<std::string>>("prompt_list"),
            BT::OutputPort<std::vector<std::string>>("clusters"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

};