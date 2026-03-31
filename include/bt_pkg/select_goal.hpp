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
    // Constructor declaration (implementation is in the .cpp file where the subscriber is created)
    SelectGoal(const std::string& name, const BT::NodeConfiguration& config);

    // Ports consumed/produced by this node.
    static BT::PortsList providedPorts()
    {
        return {
            // Control selection strategy (GENERIC, SPECIFIC, etc.)
            BT::InputPort<LogicType>("logic"),  
            
            // The name of the object we are looking for (e.g., "pillow")
            BT::InputPort<std::string>("target_class"),
            
            // Fallback exploration pose or cluster center
            BT::InputPort<geometry_msgs::msg::PoseStamped>("cluster_centroid"),
            
            // Robot's starting position used to calculate closest object
            BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"),

            // Minimum score required to accept a candidate object pose (SigLIP threshold)
            BT::InputPort<double>("similarity_threshold"),
            
            // Selected navigation target (object pose or cluster centroid).
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            
            // True when target_pose is an object goal, false when it is the centroid.
            BT::OutputPort<bool>("is_object_goal")
        };
    }

    // Chooses the best target and writes outputs to the blackboard.
    BT::NodeStatus tick() override;

private:
    // ROS 2 Subscription components
    rclcpp::Subscription<yolo11_seg_interfaces::msg::SemanticObjectArray>::SharedPtr map_sub_;
    yolo11_seg_interfaces::msg::SemanticObjectArray::SharedPtr latest_map_;
    std::mutex map_mutex_;
};