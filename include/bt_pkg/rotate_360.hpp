#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

// Stateful BT action that rotates the robot in place for approximately one full turn.
class Rotate360 : public BT::StatefulActionNode
{
public:
    Rotate360(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            // Rotation step parameter kept for BT configurability/future strategies.
            BT::InputPort<double>("angle_increment", 10.0, "Angle increment in degrees for rotation steps")
        };
    }

    // Stateful BT lifecycle methods.
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // Publisher used to command angular velocity on /cmd_vel.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Timestamp when rotation starts.
    rclcpp::Time start_time_;
    // Tracked rotation amount in degrees.
    double total_rotation_deg_ = 0.0;
    // Constant angular speed for the time-based rotation.
    double angular_velocity_ = 0.5; // rad/s (approximately 28.6 deg/s)
    // Completion flag for internal state tracking.
    bool rotation_complete_ = false;
};
