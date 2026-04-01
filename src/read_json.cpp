#include "bt_pkg/read_json.hpp"
#include <iostream>

BT::NodeStatus ReadJson::tick()
{
    // Read input path to the JSON command file.
    std::string file_path;
    if (!getInput("file_path", file_path)) {
        std::cerr << "[ReadJson] Error: Missing file_path input" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Open file stream.
    std::ifstream f(file_path);
    if (!f.is_open()) {
        std::cerr << "[ReadJson] Error: Could not open file at " << file_path << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        // Parse JSON document.
        json data = json::parse(f);

        // Extract command-level goal class used to filter objects from the clustered-map topic.
        std::string goal_class = "";
        if (data.contains("goal") && data["goal"].is_string()) {
            goal_class = data["goal"].get<std::string>();
        }
        setOutput("goal_class", goal_class);

        // Extract high-level action string.
        std::string action = "";
        if (data.contains("action") && data["action"].is_string()) {
            action = data["action"].get<std::string>();
        }
        setOutput("action", action);

        std::string logic_str = "";
        LogicType logic_value = LogicType::GENERIC_OBJECT; // Default value.
        if (data.contains("logic") && data["logic"].is_string()) {
            logic_str = data["logic"].get<std::string>();
            if (logic_str == "GENERIC_OBJECT") {
                logic_value = LogicType::GENERIC_OBJECT;
            } else if (logic_str == "GENERIC_OBJECT_SPECIFIC_LOCATION") {
                logic_value = LogicType::GENERIC_OBJECT_SPECIFIC_LOCATION;
            } else if (logic_str == "SPECIFIC_OBJECT_WITH_FEATURES") {
                logic_value = LogicType::SPECIFIC_OBJECT_WITH_FEATURES;
            } else {
                std::cerr << "[ReadJson] Warning: Unrecognized logic type '" << logic_str << "'. Defaulting to GENERIC_OBJECT." << std::endl;
            }
        }
        setOutput("logic", logic_value);
        
        // Extract selected cluster id from command.
        int cluster_id_value = -1;
        
        if (data.contains("cluster_info") && data["cluster_info"].is_object()) {
            const auto& ci = data["cluster_info"];
            if (ci.contains("cluster_id")) {
                cluster_id_value = ci["cluster_id"].get<int>();
            }
        }
        // cluster_id is mandatory for validity.
        if (cluster_id_value < 0) {
            std::cerr << "[ReadJson] Error: Missing cluster_info.cluster_id" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        setOutput("cluster", cluster_id_value);

                std::cout << "[ReadJson] Successfully parsed command fields. goal='" << goal_class
                << "' cluster=" << cluster_id_value << " action='" << action << "'." << std::endl;

        return BT::NodeStatus::SUCCESS;

    } catch (const json::parse_error& e) {
        std::cerr << "[ReadJson] JSON Parse Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "[ReadJson] Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}