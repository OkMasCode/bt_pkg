#include "bt_pkg/find_approach_pose.hpp"
#include "yolo11_seg_interfaces/srv/get_approach_pose.hpp"
#include <rclcpp/rclcpp.hpp>

FindApproachPose::FindApproachPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    if (!config.blackboard->get("node", node_)) {
        throw BT::RuntimeError("FindApproachPose requires a 'node' pointer in the blackboard!");
    }
    // Isolated node so spin_until_future_complete never deadlocks with the main executor
    client_node_ = rclcpp::Node::make_shared("find_approach_pose_client_node");
    client_ = client_node_->create_client<yolo11_seg_interfaces::srv::GetApproachPose>(
        "/vision/get_approach_pose");
}

BT::NodeStatus FindApproachPose::tick()
{
    int room_id;
    if (!getInput("target_cluster", room_id)) {
        RCLCPP_ERROR(node_->get_logger(), "[PLAN] Cannot plan approach: missing target_cluster input");
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("goal_pose", goal_pose)) {
        RCLCPP_ERROR(node_->get_logger(), "[PLAN] Cannot plan approach: missing goal_pose input");
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped start_pose;
    if (!getInput("start_pose", start_pose)) {
        RCLCPP_ERROR(node_->get_logger(), "[PLAN] Cannot plan approach: missing start_pose input");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "[PLAN] Computing a safe approach pose near the goal in cluster %d", room_id);

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "[PLAN] Approach-pose service not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<yolo11_seg_interfaces::srv::GetApproachPose::Request>();
    request->room_id = room_id;
    request->goal_pose = goal_pose;
    request->start_pose = start_pose;

    auto future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "[PLAN] Approach-pose service call failed");
        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "[PLAN] No safe approach pose could be found for cluster %d", room_id);
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header.frame_id = "map"; 
    approach_pose.header.stamp = node_->now(); // Use main node's clock for accurate TF sync
    approach_pose.pose = response->approach_pose.pose;
    setOutput("approach_pose", approach_pose);
    
    RCLCPP_INFO(node_->get_logger(), "[PLAN] Safe approach pose ready at (%.2f, %.2f)",
                approach_pose.pose.position.x, approach_pose.pose.position.y);

    return BT::NodeStatus::SUCCESS;
}