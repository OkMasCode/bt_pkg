#include "bt_pkg/rotate_360.hpp"

BT::NodeStatus Rotate360::onStart()
{
    // Retrieve ROS node from blackboard.
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Create velocity publisher used during rotation.
    cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("/jackal/platform/cmd_vel", 10);
    
    // Validate configured input port (currently not used by time-based implementation).
    auto angle_inc = getInput<double>("angle_increment");
    if (!angle_inc)
    {
        RCLCPP_ERROR(node->get_logger(), "[SCAN] Cannot rotate: missing angle_increment");
        return BT::NodeStatus::FAILURE;
    }
    
    // Initialize rotation state.
    start_time_ = node->now();
    total_rotation_deg_ = 0.0;
    rotation_complete_ = false;
    
    RCLCPP_INFO(node->get_logger(), "[SCAN] Spinning in place to look around (%.2f rad/s)",
                angular_velocity_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Rotate360::onRunning()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Compute elapsed time since start.
    auto current_time = node->now();
    double elapsed_sec = (current_time - start_time_).seconds();
    
    // Convert elapsed time to estimated rotation angle.
    total_rotation_deg_ = elapsed_sec * angular_velocity_ * (180.0 / M_PI);
    
    if (total_rotation_deg_ >= 360.0)
    {
        // Full turn reached: stop and return success.
        auto stop_msg = geometry_msgs::msg::TwistStamped();
        stop_msg.twist.angular.z = 0.0;
        cmd_vel_publisher_->publish(stop_msg);
        
        RCLCPP_INFO(node->get_logger(), "[SCAN] Full 360 scan complete");
        rotation_complete_ = true;
        return BT::NodeStatus::SUCCESS;
    }
    
    // Keep rotating in place (counter-clockwise, +Z).
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = angular_velocity_; // Positive = counter-clockwise
    twist_msg.header.stamp = current_time;
    
    cmd_vel_publisher_->publish(twist_msg);
    
    // Periodic progress logging.
    if (static_cast<int>(elapsed_sec) % 2 == 0 && elapsed_sec > 0.1)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "[SCAN] Scanning... %.0f / 360 deg", total_rotation_deg_);
    }
    
    return BT::NodeStatus::RUNNING;
}

void Rotate360::onHalted()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Halt callback: stop robot immediately.
    auto stop_msg = geometry_msgs::msg::TwistStamped();
    stop_msg.twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(stop_msg);
    
    RCLCPP_WARN(node->get_logger(), "[SCAN] Scan interrupted at %.0f deg - stopping", total_rotation_deg_);
}
