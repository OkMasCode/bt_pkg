#include "bt_pkg/select_goal.hpp"

#include <algorithm>
#include <limits>
#include <sstream>

using ClusteredMapArrayMsg = yolo11_seg_interfaces::msg::ClusteredMapObjectArray;

void SelectGoal::ensureSubscription(
    const rclcpp::Node::SharedPtr& node,
    const std::string& topic_name)
{
    if (map_sub_ && subscribed_topic_ == topic_name) {
        return;
    }

    subscribed_topic_ = topic_name;
    map_sub_ = node->create_subscription<ClusteredMapArrayMsg>(
        subscribed_topic_,
        rclcpp::QoS(10),
        [this](const ClusteredMapArrayMsg::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            latest_map_msg_ = msg;
        });

    RCLCPP_INFO(node->get_logger(), "SelectGoal subscribed to clustered-map topic: %s", subscribed_topic_.c_str());
}

bool SelectGoal::hasMapSnapshot() const
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    return static_cast<bool>(latest_map_msg_);
}

BT::NodeStatus SelectGoal::tick()
{
    // Retrieve ROS node handle from the BT blackboard for logging and runtime context.
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }

    LogicType logic;
    std::string goal_class;
    std::string clustered_map_topic = "/vision/clustered_map_v6";
    int cluster;
    geometry_msgs::msg::PoseStamped start_pose;
    double similarity_threshold = 8.0;

    if (!getInput("logic", logic)) {
        RCLCPP_ERROR(node->get_logger(), "Missing logic input");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput("goal_class", goal_class)) {
        RCLCPP_ERROR(node->get_logger(), "Missing goal_class input");
        return BT::NodeStatus::FAILURE;
    }
    getInput("clustered_map_topic", clustered_map_topic);
    if (!getInput("cluster", cluster)) {
        RCLCPP_ERROR(node->get_logger(), "Missing cluster input");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput("start_pose", start_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Missing start_pose input");
        return BT::NodeStatus::FAILURE;
    }
    // Optional threshold is retained for BT compatibility.
    getInput("similarity_threshold", similarity_threshold);

    ensureSubscription(node, clustered_map_topic);
    if (!hasMapSnapshot()) {
        RCLCPP_WARN(node->get_logger(), "No clustered-map message available yet on %s", clustered_map_topic.c_str());
        return BT::NodeStatus::FAILURE;
    }

    ClusteredMapArrayMsg::SharedPtr map_msg;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_msg = latest_map_msg_;
    }

    // Build candidate sets from the latest clustered map.
    std::vector<std::string> candidates_ids;
    std::vector<double> similarity_scores;
    std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
    std::vector<int> cluster_ids;

    geometry_msgs::msg::PoseStamped cluster_centroid;
    cluster_centroid.header.frame_id = map_msg->frame_id.empty() ? "map" : map_msg->frame_id;
    cluster_centroid.header.stamp = map_msg->stamp;
    cluster_centroid.pose.orientation.w = 1.0;

    std::vector<double> cluster_dimensions = {0.0, 0.0};
    bool cluster_found = false;

    for (const auto& obj : map_msg->objects) {
        // Capture selected cluster geometry once for downstream exploration nodes.
        if (!cluster_found && obj.cluster == cluster) {
            cluster_centroid.pose.position.x = obj.cluster_centroid.x;
            cluster_centroid.pose.position.y = obj.cluster_centroid.y;
            cluster_centroid.pose.position.z = obj.cluster_centroid.z;
            cluster_dimensions[0] = obj.cluster_dimensions.bounding_box.width;
            cluster_dimensions[1] = obj.cluster_dimensions.bounding_box.length;
            cluster_found = true;
        }

        if (!goal_class.empty() && obj.class_name != goal_class) {
            continue;
        }

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = map_msg->frame_id.empty() ? "map" : map_msg->frame_id;
        goal_pose.header.stamp = map_msg->stamp;
        goal_pose.pose.position.x = obj.coords.x;
        goal_pose.pose.position.y = obj.coords.y;
        goal_pose.pose.position.z = obj.coords.z;
        goal_pose.pose.orientation.w = 1.0;

        candidates_ids.push_back(obj.id);
        similarity_scores.push_back(obj.similarity);
        goal_poses.push_back(goal_pose);
        cluster_ids.push_back(obj.cluster);
    }

    if (!cluster_found) {
        RCLCPP_WARN(node->get_logger(), "Selected cluster %d not present in clustered-map topic", cluster);
        return BT::NodeStatus::FAILURE;
    }

    setOutput("candidates_ids", candidates_ids);
    setOutput("similarity_scores", similarity_scores);
    setOutput("goal_poses", goal_poses);
    setOutput("cluster_ids", cluster_ids);
    setOutput("cluster_centroid", cluster_centroid);
    setOutput("cluster_dimensions", cluster_dimensions);

    // If clustered-map has no matching class candidates, fall back to exploring the selected cluster.
    if (candidates_ids.empty() || goal_poses.empty()) {
        RCLCPP_WARN(node->get_logger(), "No objects found for class '%s'. Using cluster centroid.", goal_class.c_str());
        setOutput("target_pose", cluster_centroid);
        setOutput("is_object_goal", false);
        return BT::NodeStatus::SUCCESS;
    }

    if (logic == LogicType::GENERIC_OBJECT) {
        RCLCPP_INFO(node->get_logger(), "Logic: GENERIC_OBJECT - selecting closest object of class '%s'.", goal_class.c_str());

        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < goal_poses.size(); ++i) {
            double dx = goal_poses[i].pose.position.x - start_pose.pose.position.x;
            double dy = goal_poses[i].pose.position.y - start_pose.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        RCLCPP_INFO(node->get_logger(), "Closest object found at distance: %.2f", min_dist);
        setOutput("target_pose", goal_poses[closest_idx]);
        setOutput("is_object_goal", true);
        return BT::NodeStatus::SUCCESS;

    } else if (logic == LogicType::GENERIC_OBJECT_SPECIFIC_LOCATION) {
        RCLCPP_INFO(node->get_logger(), "Logic: GENERIC_OBJECT_SPECIFIC_LOCATION - preferring selected cluster %d.", cluster);

        std::vector<size_t> in_cluster_indices;
        for (size_t i = 0; i < goal_poses.size(); ++i) {
            if (i < cluster_ids.size() && cluster_ids[i] == cluster) {
                in_cluster_indices.push_back(i);
            }
        }

        if (in_cluster_indices.empty()) {
            RCLCPP_WARN(node->get_logger(), "No '%s' objects found in cluster %d. Using centroid.", goal_class.c_str(), cluster);
            setOutput("target_pose", cluster_centroid);
            setOutput("is_object_goal", false);
            return BT::NodeStatus::SUCCESS;
        }

        size_t closest_idx = in_cluster_indices[0];
        double min_dist = std::numeric_limits<double>::max();
        for (size_t idx : in_cluster_indices) {
            double dx = goal_poses[idx].pose.position.x - start_pose.pose.position.x;
            double dy = goal_poses[idx].pose.position.y - start_pose.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = idx;
            }
        }

        RCLCPP_INFO(node->get_logger(), "Selected closest object in cluster %d at distance: %.2f", cluster, min_dist);
        setOutput("target_pose", goal_poses[closest_idx]);
        setOutput("is_object_goal", true);
        return BT::NodeStatus::SUCCESS;

    } else if (logic == LogicType::SPECIFIC_OBJECT_WITH_FEATURES) {
        RCLCPP_INFO(node->get_logger(),
                    "Logic: SPECIFIC_OBJECT_WITH_FEATURES - selecting highest-similarity object for class '%s'.",
                    goal_class.c_str());

        size_t best_idx = 0;
        double best_similarity = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < similarity_scores.size(); ++i) {
            if (similarity_scores[i] > best_similarity) {
                best_similarity = similarity_scores[i];
                best_idx = i;
            }
        }

        if (best_similarity < similarity_threshold) {
            RCLCPP_INFO(node->get_logger(),
                        "Best similarity %.3f is below threshold %.3f. Using cluster centroid.",
                        best_similarity,
                        similarity_threshold);
            setOutput("target_pose", cluster_centroid);
            setOutput("is_object_goal", false);
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(node->get_logger(),
                    "Selected object '%s' with highest similarity %.3f (threshold %.3f).",
                    candidates_ids[best_idx].c_str(),
                    best_similarity,
                    similarity_threshold);
        setOutput("target_pose", goal_poses[best_idx]);
        setOutput("is_object_goal", true);
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node->get_logger(), "Unknown logic type. Defaulting to cluster centroid.");
    setOutput("target_pose", cluster_centroid);
    setOutput("is_object_goal", false);
    return BT::NodeStatus::SUCCESS;
}