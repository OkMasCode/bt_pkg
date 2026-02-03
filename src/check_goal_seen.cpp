#include "bt_pkg/check_goal_seen.hpp"

BT::NodeStatus CallCheckCandidates::tick()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }

    std::vector<std::string> candidates_ids;
    std::vector<double> similarity_scores;
    std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
    geometry_msgs::msg::PoseStamped cluster_centroid;
    double similarity_threshold = 0.0;

    if (!getInput("candidates_ids", candidates_ids)) {
        RCLCPP_ERROR(node->get_logger(), "Missing candidates_ids input");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput("similarity_scores", similarity_scores)) {
        RCLCPP_ERROR(node->get_logger(), "Missing similarity_scores input");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput("goal_poses", goal_poses)) {
        RCLCPP_ERROR(node->get_logger(), "Missing goal_poses input");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput("cluster_centroid", cluster_centroid)) {
        RCLCPP_ERROR(node->get_logger(), "Missing cluster_centroid input");
        return BT::NodeStatus::FAILURE;
    }
    getInput("similarity_threshold", similarity_threshold);

    if (candidates_ids.empty() || similarity_scores.empty() || goal_poses.empty()) {
        RCLCPP_WARN(node->get_logger(), "Empty candidates/similarity/poses. Using cluster centroid.");
        setOutput("target_pose", cluster_centroid);
        return BT::NodeStatus::SUCCESS;
    }

    if (similarity_scores.size() != goal_poses.size()) {
        RCLCPP_WARN(node->get_logger(), "Mismatch sizes: similarity_scores=%zu, goal_poses=%zu. Using cluster centroid.",
                    similarity_scores.size(), goal_poses.size());
        setOutput("target_pose", cluster_centroid);
        return BT::NodeStatus::SUCCESS;
    }

    size_t best_idx = 0;
    double best_score = similarity_scores[0];
    for (size_t i = 1; i < similarity_scores.size(); ++i) {
        if (similarity_scores[i] > best_score) {
            best_score = similarity_scores[i];
            best_idx = i;
        }
    }

    if (best_score >= similarity_threshold) {
        RCLCPP_INFO(node->get_logger(), "Selecting goal pose (score=%.2f >= %.2f)",
                    best_score, similarity_threshold);
        setOutput("target_pose", goal_poses[best_idx]);
        setOutput("is_object_goal", true);
    } else {
        RCLCPP_INFO(node->get_logger(), "Selecting cluster centroid (score=%.2f < %.2f)",
                    best_score, similarity_threshold);
        setOutput("target_pose", cluster_centroid);
        setOutput("is_object_goal", false);
    }

    return BT::NodeStatus::SUCCESS;
}