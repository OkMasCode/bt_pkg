#include "bt_pkg/check_goal_seen.hpp"

BT::NodeStatus CallCheckCandidates::onStart()
{
    // 1. Get the ROS Node from the Blackboard
    // (This was set in main.cpp: blackboard->set<rclcpp::Node::SharedPtr>("node", node))
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }

    // 2. Initialize the Client (only once)
    if (!client_) {
        client_ = node->create_client<CheckCandidates>("check_candidates");
    }

    // 3. Prepare the Request
    auto request = std::make_shared<CheckCandidates::Request>();

    // Get inputs from ports
    if (!getInput("candidates_ids", request->candidates_ids)) {
        RCLCPP_ERROR(node->get_logger(), "Missing candidates_ids input");
        return BT::NodeStatus::FAILURE;
    }

    // 4. Send the Request
    // Wait max 1 second for the server to be available
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "check_candidates service not available");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node->get_logger(), "Checking %ld candidates...", 
                request->candidates_ids.size());

    // Send asynchronously so we don't freeze the tree
    future_result_ = client_->async_send_request(request).share();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallCheckCandidates::onRunning()
{
    // 1. Check if the response has arrived
    if (future_result_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING; // Still waiting...
    }

    // 2. Get the result
    auto response = future_result_.get();
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    if (response->match_found) {
        RCLCPP_INFO(node->get_logger(), "Match Found! ID: %s (Score: %.2f)", 
                    response->best_candidate_id.c_str(), response->max_score);

        // 3. Create the PoseStamped for Navigation
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = node->now();
        goal_pose.header.frame_id = "map"; // Assuming coordinates are in map frame

        // Map the service response (x, y) to the Pose
        goal_pose.pose.position.x = response->position.x;
        goal_pose.pose.position.y = response->position.y;
        goal_pose.pose.position.z = 0.0;
        
        // Orientation: standard facing forward (w=1.0)
        goal_pose.pose.orientation.w = 1.0; 
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;

        // 4. Write to Output Port (so NavigateToPose can read it)
        setOutput("target_pose", goal_pose);

        return BT::NodeStatus::SUCCESS;
    } 
    else {
        RCLCPP_WARN(node->get_logger(), "No match found above threshold.");
        return BT::NodeStatus::FAILURE; // This triggers the Fallback to "Exploration"
    }
}

void CallCheckCandidates::onHalted()
{
    // Optional: Cancel the request if the tree stops this node
}