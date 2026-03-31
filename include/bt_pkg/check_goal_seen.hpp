#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// Include your custom map message
#include "yolo11_seg_interfaces/msg/semantic_object_array.hpp"

class CheckGoalSeen : public BT::ConditionNode
{
public:
    CheckGoalSeen(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("command_file_path"),
            BT::InputPort<double>("similarity_threshold"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
        };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    
    // Subscribe directly to the live map
    rclcpp::Subscription<yolo11_seg_interfaces::msg::SemanticObjectArray>::SharedPtr map_sub_;
    
    // Memory to store the latest map received in the background
    yolo11_seg_interfaces::msg::SemanticObjectArray::SharedPtr latest_map_;
    std::mutex map_mutex_;

    // Helper to read the goal from the JSON file
    std::string getGoalFromJson(const std::string& file_path);
};