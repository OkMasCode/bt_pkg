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

        // --- EXTRACT: Candidate IDs ---
        // New schema: "goal_objects" is an array of objects { id, coords }
        std::vector<std::string> candidates;
        if (data.contains("goal_objects") && data["goal_objects"].is_array()) {
            for (const auto& item : data["goal_objects"]) {
                if (item.is_object() && item.contains("id")) {
                    candidates.push_back(item["id"].get<std::string>());
                }
            }
        }
        setOutput("candidats_ids", candidates);

        // --- EXTRACT: Clip Prompt ---
        // Maps to "clip_prompt" (which is a list of strings)
        std::vector<std::string> prompts;
        if (data.contains("clip_prompt") && data["clip_prompt"].is_array()) {
            for (const auto& item : data["clip_prompt"]) {
                prompts.push_back(item.get<std::string>());
            }
        } else if (data.contains("prompt") && data["prompt"].is_string()) {
            // Fallback: single prompt string if clip_prompt not provided
            prompts.push_back(data["prompt"].get<std::string>());
        }
        setOutput("prompt", prompts);


        // --- EXTRACT: Cluster Centroid Coordinates ---
        // New schema: cluster_info contains centroid under "coords"
        geometry_msgs::msg::Pose centroid;
        bool have_centroid = false;
        if (data.contains("cluster_info") && data["cluster_info"].is_object()) {
            const auto& ci = data["cluster_info"];
            if (ci.contains("coords") && ci["coords"].is_object()) {
                const auto& coords = ci["coords"];
                centroid.position.x = coords.value("x", 0.0);
                centroid.position.y = coords.value("y", 0.0);
                centroid.position.z = coords.value("z", 0.0);
                centroid.orientation.w = 1.0;
                have_centroid = true;
            }
        }
        if (!have_centroid) {
            std::cerr << "[ReadJson] Error: Missing cluster_info.coords centroid" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        setOutput("cluster_id", centroid);

        std::cout << "[ReadJson] Successfully parsed command. Found " 
              << candidates.size() << " candidates and set cluster centroid." << std::endl;

        return BT::NodeStatus::SUCCESS;

    } catch (const json::parse_error& e) {
        std::cerr << "[ReadJson] JSON Parse Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "[ReadJson] Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}