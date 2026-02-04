#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

class Rotate360 : public BT::StatefulActionNode
{
public:
    Rotate360(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("angle_increment", 10.0, "Angle increment in degrees for rotation steps")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Time start_time_;
    double total_rotation_deg_ = 0.0;
    double angular_velocity_ = 0.5; // rad/s (approximately 28.6 deg/s)
    bool rotation_complete_ = false;
};
