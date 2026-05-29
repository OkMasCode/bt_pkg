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

        const bool bring_back = (action == "bring_back_object");

        // Narrate the post-arrival decision once, only when it changes.
        const int state = bring_back ? 1 : 0;
        if (state != last_logged_) {
            last_logged_ = state;
            auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            if (node) {
                if (bring_back) {
                    RCLCPP_INFO(node->get_logger(),
                        "[DECISION] Action is bring_back_object - returning to the start pose");
                } else {
                    RCLCPP_INFO(node->get_logger(),
                        "[DECISION] No bring-back requested - mission ends at the goal");
                }
            }
        }

        // Pass only when current action is bring_back_object.
        return bring_back ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    // -1 = not yet logged, 0 = last logged "no bring-back", 1 = last logged "bring back".
    int last_logged_ = -1;
};
