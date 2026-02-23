#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Synchronous BT action that captures the robot pose from TF and stores it as start_pose.
class GetRobotStartPose : public BT::SyncActionNode
{
public:
    GetRobotStartPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config),
        // TF buffer/listener are owned by this node and initialized once at construction.
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::System())),
        tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // TF source/topic selector (declared for tree configuration compatibility).
            BT::InputPort<std::string>("tf_topic"),
            // Robot frame to query against the map frame (e.g., base_link).
            BT::InputPort<std::string>("frame_id"),
            
            // Robot pose in map frame at query time.
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("start_pose")
        };
    }

    // Reads TF transform map->frame_id and outputs a PoseStamped.
    BT::NodeStatus tick() override;

private:
    // TF utilities used to perform transform lookups.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
