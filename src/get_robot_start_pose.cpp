#include "bt_pkg/get_robot_start_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

BT::NodeStatus GetRobotStartPose::tick()
{
    // Logger from the shared blackboard node so all BT output goes through one channel.
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    auto logger = node ? node->get_logger() : rclcpp::get_logger("GetRobotStartPose");

    // Read required frame input used for TF lookup.
    std::string frame_id;
    if (!getInput("frame_id", frame_id)) {
        RCLCPP_ERROR(logger, "[POSE] Cannot locate robot: missing frame_id input");
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

        RCLCPP_INFO(logger, "[POSE] Robot start pose captured at (%.2f, %.2f)",
                    start_pose.pose.position.x, start_pose.pose.position.y);

        return BT::NodeStatus::SUCCESS;
    }
    catch (const tf2::TransformException& e) {
        // TF lookup failures (missing frames, timeout, etc.).
        RCLCPP_ERROR(logger, "[POSE] Cannot locate robot - TF lookup failed: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
    catch (const std::exception& e) {
        // Fallback for unexpected runtime errors.
        RCLCPP_ERROR(logger, "[POSE] Cannot locate robot: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}
