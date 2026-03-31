#pragma once

#include <vector>
#include <string>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Custom message for the map subscription
#include "yolo11_seg_interfaces/msg/semantic_object_array.hpp"

#include "bt_pkg/logic_type.hpp"

// Synchronous BT action that selects a navigation target directly from the Semantic Map.
class SelectGoal : public BT::SyncActionNode
{
public:
    // Constructor declaration
    SelectGoal(const std::string& name, const BT::NodeConfiguration& config);

    // Ports consumed/produced by this node. (Declaration only!)
    static BT::PortsList providedPorts();

    // Chooses the best target and writes outputs to the blackboard.
    BT::NodeStatus tick() override;

private:
    // ROS 2 Subscription components
    rclcpp::Subscription<yolo11_seg_interfaces::msg::SemanticObjectArray>::SharedPtr map_sub_;
    yolo11_seg_interfaces::msg::SemanticObjectArray::SharedPtr latest_map_;
    std::mutex map_mutex_;
};