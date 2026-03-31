#include "bt_pkg/check_goal_seen.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

CheckGoalSeen::CheckGoalSeen(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
}

// Helper: Replicates the Python script's logic to extract the goal string
std::string CheckGoalSeen::getGoalFromJson(const std::string& file_path) {
    std::ifstream f(file_path);
    if (!f.is_open()) return "";
    try {
        nlohmann::json data = nlohmann::json::parse(f);
        if (data.contains("goal") && data["goal"].is_string()) {
            std::string goal = data["goal"];
            // Convert to lowercase to match the map logic
            std::transform(goal.begin(), goal.end(), goal.begin(), 
                           [](unsigned char c){ return std::tolower(c); });
            return goal;
        }
    } catch (const std::exception& e) {
        // Silently handle parsing errors
    }
    return "";
}

BT::NodeStatus CheckGoalSeen::tick()
{
    // 1. Initialize ROS node and subscriber on the first tick
    if (!node_) {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_) {
            throw std::runtime_error("Missing ROS Node in Blackboard");
        }

        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.durability_volatile();

        map_sub_ = node_->create_subscription<yolo11_seg_interfaces::msg::SemanticObjectArray>(
            "/vision/semantic_map_v5", qos,
            [this](const yolo11_seg_interfaces::msg::SemanticObjectArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(map_mutex_);
                latest_map_ = msg;
            });
        
        RCLCPP_INFO(node_->get_logger(), "[CheckGoalSeen] Subscribed to live map.");
    }

    // 2. Read Inputs from the Blackboard/XML
    std::string file_path = "/workspaces/ros2_ws/src/yolo11_seg_bringup/config/robot_command.json";
    getInput("command_file_path", file_path);

    double similarity_threshold = 15.0; 
    getInput("similarity_threshold", similarity_threshold);

    // 3. Determine target_class from JSON
    std::string target_class = getGoalFromJson(file_path);
    if (target_class.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
            "[CheckGoalSeen] Could not read goal from JSON. Returning FAILURE.");
        return BT::NodeStatus::FAILURE;
    }

    // 4. Safely copy the latest map
    yolo11_seg_interfaces::msg::SemanticObjectArray::SharedPtr current_map;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map = latest_map_;
    }

    if (!current_map || current_map->objects.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    // 5. Search for the best matching object
    double best_sim = -1.0;
    bool found = false;
    yolo11_seg_interfaces::msg::SemanticObject best_obj;

    for (const auto& obj : current_map->objects) {
        // Enforce lowercase comparison
        std::string obj_name = obj.name;
        std::transform(obj_name.begin(), obj_name.end(), obj_name.begin(), 
                       [](unsigned char c){ return std::tolower(c); });

        if (obj_name == target_class) {
            if (obj.similarity > best_sim) {
                best_sim = obj.similarity;
                best_obj = obj;
                found = true;
            }
        }
    }

    // 6. Evaluate the match against the threshold
    if (found && best_sim >= similarity_threshold) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = best_obj.frame;
        goal_pose.header.stamp = node_->now();
        goal_pose.pose.position.x = best_obj.pose_map.x;
        goal_pose.pose.position.y = best_obj.pose_map.y;
        goal_pose.pose.position.z = best_obj.pose_map.z;
        goal_pose.pose.orientation.w = 1.0;

        setOutput("goal_pose", goal_pose);
        
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[CheckGoalSeen] Goal '%s' SEEN! Sim: %.2f >= %.2f",
            target_class.c_str(), best_sim, similarity_threshold);
            
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}