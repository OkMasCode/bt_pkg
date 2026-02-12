#include "bt_pkg/read_json.hpp"
#include <iostream>

BT::NodeStatus ReadJson::tick()
{
    // 1. Get the file path from the Input Port
    std::string file_path;
    if (!getInput("file_path", file_path)) {
        std::cerr << "[ReadJson] Error: Missing file_path input" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // 2. Open the File
    std::ifstream f(file_path);
    if (!f.is_open()) {
        std::cerr << "[ReadJson] Error: Could not open file at " << file_path << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        // 3. Parse JSON using nlohmann library
        json data = json::parse(f);

        // --- EXTRACT: Candidate IDs + Similarity Scores ---
        // New schema: "goal_objects" is an array of objects { id, similarity_score, coords }
        std::vector<std::string> candidates;
        std::vector<double> similarity_scores;
        std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
        if (data.contains("goal_objects") && data["goal_objects"].is_array()) {
            for (const auto& item : data["goal_objects"]) {
                if (item.is_object() && item.contains("id")) {
                    candidates.push_back(item["id"].get<std::string>());
                    if (item.contains("similarity_score") && item["similarity_score"].is_number()) {
                        similarity_scores.push_back(item["similarity_score"].get<double>());
                    } else {
                        similarity_scores.push_back(0.0);
                    }

                    geometry_msgs::msg::PoseStamped goal_pose;
                    goal_pose.header.frame_id = "map";
                    goal_pose.header.stamp = rclcpp::Clock().now();
                    if (item.contains("coords") && item["coords"].is_object()) {
                        const auto& coords = item["coords"];
                        goal_pose.pose.position.x = coords["x"].get<double>();
                        goal_pose.pose.position.y = coords["y"].get<double>();
                        goal_pose.pose.position.z = coords["z"].get<double>();
                    } else {
                        goal_pose.pose.position.x = 0.0;
                        goal_pose.pose.position.y = 0.0;
                        goal_pose.pose.position.z = 0.0;
                    }
                    goal_pose.pose.orientation.w = 1.0;
                    goal_pose.pose.orientation.x = 0.0;
                    goal_pose.pose.orientation.y = 0.0;
                    goal_pose.pose.orientation.z = 0.0;
                    goal_poses.push_back(goal_pose);
                }
            }
        }
        setOutput("candidates_ids", candidates);
        setOutput("similarity_scores", similarity_scores);
        setOutput("goal_poses", goal_poses);

        // --- EXTRACT: Clip Prompts ---
        // Maps to "clip_prompts" (which is a list of strings)
        std::vector<std::string> prompts;
        if (data.contains("clip_prompts") && data["clip_prompts"].is_array()) {
            for (const auto& item : data["clip_prompts"]) {
                prompts.push_back(item.get<std::string>());
            }
        } else if (data.contains("clip_prompt") && data["clip_prompt"].is_array()) {
            for (const auto& item : data["clip_prompt"]) {
                prompts.push_back(item.get<std::string>());
            }
        } else if (data.contains("prompt") && data["prompt"].is_string()) {
            // Fallback: single prompt string if clip_prompt not provided
            prompts.push_back(data["prompt"].get<std::string>());
        }
        setOutput("prompt", prompts);


        // --- EXTRACT: Action ---
        std::string action = "";
        if (data.contains("action") && data["action"].is_string()) {
            action = data["action"].get<std::string>();
        }
        setOutput("action", action);

        // --- EXTRACT: Cluster ID and Centroid ---
        // New schema: cluster_info contains cluster_id and coords
        int cluster_id_value = -1;
        geometry_msgs::msg::PoseStamped cluster_centroid;
        std::string cluster_dimensions_str = "";
        
        if (data.contains("cluster_info") && data["cluster_info"].is_object()) {
            const auto& ci = data["cluster_info"];
            if (ci.contains("cluster_id")) {
                cluster_id_value = ci["cluster_id"].get<int>();
            }
            
            // Extract centroid coordinates
            if (ci.contains("coords") && ci["coords"].is_object()) {
                const auto& coords = ci["coords"];
                cluster_centroid.header.frame_id = "map";
                cluster_centroid.header.stamp = rclcpp::Clock().now();
                cluster_centroid.pose.position.x = coords["x"].get<double>();
                cluster_centroid.pose.position.y = coords["y"].get<double>();
                cluster_centroid.pose.position.z = coords.contains("z") ? coords["z"].get<double>() : 0.0;
                // Default orientation (identity quaternion)
                cluster_centroid.pose.orientation.w = 1.0;
                cluster_centroid.pose.orientation.x = 0.0;
                cluster_centroid.pose.orientation.y = 0.0;
                cluster_centroid.pose.orientation.z = 0.0;
            } else {
                std::cerr << "[ReadJson] Error: Missing cluster_info.coords" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            
            // Extract cluster dimensions as JSON string for future use
            if (ci.contains("dimensions")) {
                cluster_dimensions_str = ci["dimensions"].dump();
            }
        }
        if (cluster_id_value < 0) {
            std::cerr << "[ReadJson] Error: Missing cluster_info.cluster_id" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        setOutput("cluster", cluster_id_value);
        setOutput("cluster_centroid", cluster_centroid);
        setOutput("cluster_dimensions", cluster_dimensions_str);

        std::cout << "[ReadJson] Successfully parsed command. Found " 
              << candidates.size() << " candidates and set cluster centroid at (" 
              << cluster_centroid.pose.position.x << ", " 
              << cluster_centroid.pose.position.y << ", " 
              << cluster_centroid.pose.position.z << ")." << std::endl;

        return BT::NodeStatus::SUCCESS;

    } catch (const json::parse_error& e) {
        std::cerr << "[ReadJson] JSON Parse Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "[ReadJson] Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}