#include "bt_pkg/get_next_point.hpp"
#include "yolo11_seg_interfaces/srv/get_room_waypoint.hpp"
#include <rclcpp/rclcpp.hpp>

GetNextPoint::GetNextPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    // Grab the main ROS 2 node pointer from the blackboard (used for logging and clock)
    if (!config.blackboard->get("node", node_)) {
        throw BT::RuntimeError("GetNextPoint requires a 'node' pointer in the blackboard!");
    }
}

BT::NodeStatus GetNextPoint::tick()
{
    int target_cluster_id;
    if (!getInput("target_cluster", target_cluster_id)) {
        RCLCPP_ERROR(node_->get_logger(), "[EXPLORE] Cannot get next waypoint: missing target_cluster input");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "[EXPLORE] Requesting next exploration waypoint for cluster %d", target_cluster_id);

    // 1. THE FIX: Create a temporary, isolated node to bypass executor deadlocks
    auto temp_node = rclcpp::Node::make_shared("temp_waypoint_client_node");
    
    // Create the client on the TEMP node, not the main node_
    auto client = temp_node->create_client<yolo11_seg_interfaces::srv::GetRoomWaypoint>("/vision/get_room_waypoint");
    
    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "[EXPLORE] Waypoint service not available");
        return BT::NodeStatus::FAILURE;
    }

    // 2. Send the Request
    auto request = std::make_shared<yolo11_seg_interfaces::srv::GetRoomWaypoint::Request>();
    request->room_id = target_cluster_id;

    auto future = client->async_send_request(request);
    
    // 3. Spin the TEMP node. 
    // Because temp_node isn't attached to your main executor, this is 100% safe and will not deadlock.
    if (rclcpp::spin_until_future_complete(temp_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "[EXPLORE] Waypoint service call failed");
        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "[EXPLORE] No exploration waypoint available for cluster %d", target_cluster_id);
        return BT::NodeStatus::FAILURE;
    }

    // 4. Construct the Nav2 Pose
    geometry_msgs::msg::PoseStamped next_point;
    next_point.header.frame_id = "map"; 
    next_point.header.stamp = node_->now(); // Use main node's clock for accurate TF sync
    next_point.pose.position = response->waypoint;
    
    // Set a default orientation (facing straight/zero rotation)
    next_point.pose.orientation.x = 0.0;
    next_point.pose.orientation.y = 0.0;
    next_point.pose.orientation.z = 0.0;
    next_point.pose.orientation.w = 1.0; 

    // 5. Write to Blackboard
    setOutput("next_point", next_point);
    
    RCLCPP_INFO(node_->get_logger(), "[EXPLORE] Next waypoint at (%.2f, %.2f) - heading there to look around",
                next_point.pose.position.x, next_point.pose.position.y);

    return BT::NodeStatus::SUCCESS;
}