#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yolo11_seg_interfaces/msg/clustered_map_object_array.hpp"
#include <vector>
#include <mutex>

#include "bt_pkg/logic_type.hpp"
// Synchronous BT action that selects a navigation target from perception candidates.
class SelectGoal : public BT::SyncActionNode
{
public:
    SelectGoal(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
    {}

    // Ports consumed/produced by this node.
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<LogicType>("logic"),  // "object" or "explore" to control selection strategy.
            BT::InputPort<std::string>("goal_class"),
            BT::InputPort<std::string>("clustered_map_topic", "/vision/clustered_map_v6"),
            BT::InputPort<int>("cluster"), 
            BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"),

            BT::InputPort<double>("similarity_threshold"),
            
            // Topic-derived object candidates matching goal_class.
            BT::OutputPort<std::vector<std::string>>("candidates_ids"),
            BT::OutputPort<std::vector<double>>("similarity_scores"),
            BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goal_poses"),
            BT::OutputPort<std::vector<int>>("cluster_ids"),
            // Selected navigation target (object pose or cluster centroid).
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("cluster_centroid"),
            BT::OutputPort<std::vector<double>>("cluster_dimensions"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            // True when target_pose is an object goal, false when it is the centroid.
            BT::OutputPort<bool>("is_object_goal")
        };
    }

    // Chooses the best target and writes outputs to the blackboard.
    BT::NodeStatus tick() override;

private:
    void ensureSubscription(
        const rclcpp::Node::SharedPtr& node,
        const std::string& topic_name);

    bool hasMapSnapshot() const;

    yolo11_seg_interfaces::msg::ClusteredMapObjectArray::SharedPtr latest_map_msg_;
    rclcpp::Subscription<yolo11_seg_interfaces::msg::ClusteredMapObjectArray>::SharedPtr map_sub_;
    std::string subscribed_topic_;
    mutable std::mutex map_mutex_;
};