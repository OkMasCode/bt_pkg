#include "bt_pkg/rotate_360.hpp"

BT::NodeStatus Rotate360::onStart()
{
    // Get the ROS node from blackboard
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Create publisher for cmd_vel
    cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Get angle increment (not used in simple time-based approach, but available for future)
    auto angle_inc = getInput<double>("angle_increment");
    if (!angle_inc)
    {
        RCLCPP_ERROR(node->get_logger(), "Missing angle_increment");
        return BT::NodeStatus::FAILURE;
    }
    
    // Record start time
    start_time_ = node->now();
    total_rotation_deg_ = 0.0;
    rotation_complete_ = false;
    
    RCLCPP_INFO(node->get_logger(), "Starting 360° rotation exploration (angular_velocity: %.2f rad/s)", 
                angular_velocity_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Rotate360::onRunning()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Calculate time elapsed
    auto current_time = node->now();
    double elapsed_sec = (current_time - start_time_).seconds();
    
    // Calculate how much we've rotated (in degrees)
    total_rotation_deg_ = elapsed_sec * angular_velocity_ * (180.0 / M_PI);
    
    if (total_rotation_deg_ >= 360.0)
    {
        // Stop rotation
        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(stop_msg);
        
        RCLCPP_INFO(node->get_logger(), "360° rotation complete! Total rotation: %.1f degrees", 
                    total_rotation_deg_);
        rotation_complete_ = true;
        return BT::NodeStatus::SUCCESS;
    }
    
    // Continue rotating
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = angular_velocity_; // Positive = counter-clockwise
    
    cmd_vel_publisher_->publish(twist_msg);
    
    // Log progress every 2 seconds
    if (static_cast<int>(elapsed_sec) % 2 == 0 && elapsed_sec > 0.1)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "Rotating... %.1f / 360.0 degrees", total_rotation_deg_);
    }
    
    return BT::NodeStatus::RUNNING;
}

void Rotate360::onHalted()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Stop the robot immediately
    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(stop_msg);
    
    RCLCPP_WARN(node->get_logger(), "Rotate360 halted at %.1f degrees", total_rotation_deg_);
}
