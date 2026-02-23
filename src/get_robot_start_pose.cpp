#include "bt_pkg/get_robot_start_pose.hpp"
#include <iostream>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

BT::NodeStatus GetRobotStartPose::tick()
{
    // Read required frame input used for TF lookup.
    std::string frame_id;
    if (!getInput("frame_id", frame_id)) {
        std::cerr << "[GetRobotStartPose] Error: Missing frame_id input" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        // Query transform from map frame to the robot frame at latest available time.
        auto transform = tf_buffer_->lookupTransform("map", frame_id, tf2::TimePointZero);

        // Build output pose message in map frame.
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = "map";
        start_pose.header.stamp = transform.header.stamp;
        
        // Copy translation component.
        start_pose.pose.position.x = transform.transform.translation.x;
        start_pose.pose.position.y = transform.transform.translation.y;
        start_pose.pose.position.z = transform.transform.translation.z;
        
        // Copy orientation component.
        start_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ));

        // Publish pose to BT blackboard output.
        setOutput("start_pose", start_pose);

        std::cout << "[GetRobotStartPose] Successfully captured robot start pose at (" 
                  << start_pose.pose.position.x << ", " 
                  << start_pose.pose.position.y << ", " 
                  << start_pose.pose.position.z << ")" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
    catch (const tf2::TransformException& e) {
        // TF lookup failures (missing frames, timeout, etc.).
        std::cerr << "[GetRobotStartPose] TF Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    catch (const std::exception& e) {
        // Fallback for unexpected runtime errors.
        std::cerr << "[GetRobotStartPose] Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}
