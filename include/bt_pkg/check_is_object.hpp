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

        // This condition is ticked continuously in the reactive loop, so only narrate
        // the branch decision when it actually changes.
        if (static_cast<int>(is_object) != last_logged_) {
            last_logged_ = static_cast<int>(is_object);
            auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            if (node) {
                if (is_object) {
                    RCLCPP_INFO(node->get_logger(),
                        "[DECISION] Goal is a known object - going straight to it");
                } else {
                    RCLCPP_INFO(node->get_logger(),
                        "[DECISION] Goal not found yet - exploring the cluster to locate it");
                }
            }
        }

        // Succeeds only when target is an object.
        return is_object ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    // -1 = not yet logged, 0 = last logged "not object", 1 = last logged "object".
    int last_logged_ = -1;
};
