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
        // Maps to "goal_objects" in your uploaded JSON
        std::vector<std::string> candidates;
        if (data.contains("goal_objects")) {
            for (const auto& item : data["goal_objects"]) {
                candidates.push_back(item.get<std::string>());
            }
        }
        setOutput("candidate_ids", candidates);


        // --- EXTRACT: Clip Prompt ---
        // Maps to "clip_prompt" (which is a list of strings)
        std::vector<std::string> prompts;
        if (data.contains("clip_prompt")) {
            for (const auto& item : data["clip_prompt"]) {
                prompts.push_back(item.get<std::string>());
            }
        }
        setOutput("prompt", prompts);


        // --- EXTRACT: Clusters (Alternatives) ---
        // Maps to "alternatives" -> "coords" in your uploaded JSON
        std::vector<geometry_msgs::msg::Pose> clusters;
        if (data.contains("alternatives")) {
            for (const auto& item : data["alternatives"]) {
                if (item.contains("coords")) {
                    auto coords = item["coords"];
                    geometry_msgs::msg::Pose p;
                    
                    // JSON: { "x": 4.5, "y": 2.5, "z": 0.0 }
                    p.position.x = coords.value("x", 0.0);
                    p.position.y = coords.value("y", 0.0);
                    p.position.z = coords.value("z", 0.0);
                    
                    // Default orientation (facing forward)
                    p.orientation.w = 1.0; 
                    
                    clusters.push_back(p);
                }
            }
        }
        setOutput("clusters", clusters);

        std::cout << "[ReadJson] Successfully parsed command. Found " 
                  << candidates.size() << " candidates and " 
                  << clusters.size() << " clusters." << std::endl;

        return BT::NodeStatus::SUCCESS;

    } catch (const json::parse_error& e) {
        std::cerr << "[ReadJson] JSON Parse Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "[ReadJson] Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}