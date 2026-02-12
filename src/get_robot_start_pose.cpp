#include "bt_pkg/get_robot_start_pose.hpp"
#include <iostream>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

BT::NodeStatus GetRobotStartPose::tick()
{
    // Get input ports
    std::string frame_id;
    if (!getInput("frame_id", frame_id)) {
        std::cerr << "[GetRobotStartPose] Error: Missing frame_id input" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        // Try to get the transform from map to the robot's frame
        auto transform = tf_buffer_->lookupTransform("map", frame_id, tf2::TimePointZero);

        // Create PoseStamped from the transform
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = "map";
        start_pose.header.stamp = transform.header.stamp;
        
        // Extract position from transform
        start_pose.pose.position.x = transform.transform.translation.x;
        start_pose.pose.position.y = transform.transform.translation.y;
        start_pose.pose.position.z = transform.transform.translation.z;
        
        // Extract orientation from transform
        start_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ));

        // Set the output port
        setOutput("start_pose", start_pose);

        std::cout << "[GetRobotStartPose] Successfully captured robot start pose at (" 
                  << start_pose.pose.position.x << ", " 
                  << start_pose.pose.position.y << ", " 
                  << start_pose.pose.position.z << ")" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
    catch (const tf2::TransformException& e) {
        std::cerr << "[GetRobotStartPose] TF Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << "[GetRobotStartPose] Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}
