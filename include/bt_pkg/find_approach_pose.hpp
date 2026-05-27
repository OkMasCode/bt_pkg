#pragma once

#include "behaviortree_cpp/action_node.h" // Note: use behaviortree_cpp_v3/action_node.h if on older BT.CPP
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include "yolo11_seg_interfaces/srv/get_approach_pose.hpp"

class FindApproachPose : public BT::SyncActionNode
{
public:
    FindApproachPose(const std::string& name, const BT::NodeConfiguration& config);

    // 1. Update the ports to match the new architecture
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("target_cluster", "The ID of the room where the object is"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Pose of the object to approach"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose", "Pose of the robot to start from"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_pose", "Pose to reach")
        };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Client<yolo11_seg_interfaces::srv::GetApproachPose>::SharedPtr client_;
};
