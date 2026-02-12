#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class GetRobotStartPose : public BT::SyncActionNode
{
public:
    GetRobotStartPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config),
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::System())),
        tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // Inputs
            BT::InputPort<std::string>("tf_topic"),
            BT::InputPort<std::string>("frame_id"),
            
            // Outputs
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("start_pose")
        };
    }

    BT::NodeStatus tick() override;

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
