#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class FindApproachPose : public BT::SyncActionNode
{
public:
    FindApproachPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
            BT::InputPort<std::string>("costmap_topic"),
            BT::InputPort<double>("min_radius"),
            BT::InputPort<double>("max_radius"),
            BT::InputPort<double>("radius_step"),
            BT::InputPort<double>("angle_step_deg"),
            BT::InputPort<int>("free_threshold"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_pose")
        };
    }

    BT::NodeStatus tick() override;

private:
    void ensureSubscription(const rclcpp::Node::SharedPtr& node, const std::string& topic);
    bool isFree(const nav_msgs::msg::OccupancyGrid& grid, double wx, double wy, int free_threshold) const;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_costmap_;
    std::string last_topic_;
};
