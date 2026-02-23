#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Synchronous BT action that finds a nearby free pose from which to approach a goal.
class FindApproachPose : public BT::SyncActionNode
{
public:
    FindApproachPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // Target pose to approach.
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
            // Costmap topic used to evaluate free/occupied cells.
            BT::InputPort<std::string>("costmap_topic"),
            // Radial sampling bounds and increments around goal_pose.
            BT::InputPort<double>("min_radius"),
            BT::InputPort<double>("max_radius"),
            BT::InputPort<double>("radius_step"),
            // Angular sampling step in degrees.
            BT::InputPort<double>("angle_step_deg"),
            // Maximum occupancy value considered free.
            BT::InputPort<int>("free_threshold"),
            // Output approach pose oriented toward goal_pose.
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_pose")
        };
    }

    // Samples candidate points and outputs the first valid approach pose.
    BT::NodeStatus tick() override;

private:
    // Ensures a subscription exists for the selected costmap topic.
    void ensureSubscription(const rclcpp::Node::SharedPtr& node, const std::string& topic);
    // Checks whether a world coordinate maps to a free costmap cell.
    bool isFree(const nav_msgs::msg::OccupancyGrid& grid, double wx, double wy, int free_threshold) const;

    // Latest costmap subscription and cached message.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_costmap_;
    // Last subscribed topic, used to avoid unnecessary re-subscription.
    std::string last_topic_;
};
