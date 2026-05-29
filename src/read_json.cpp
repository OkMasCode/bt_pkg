#include "bt_pkg/read_json.hpp"
#include <rclcpp/rclcpp.hpp>

BT::NodeStatus ReadJson::tick()
{
    // Logger from the shared blackboard node so all BT output goes through one channel.
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    auto logger = node ? node->get_logger() : rclcpp::get_logger("ReadJson");

    // Read input path to the JSON command file.
    std::string file_path;
    if (!getInput("file_path", file_path)) {
        RCLCPP_ERROR(logger, "[COMMAND] Cannot read mission: missing file_path input");
        return BT::NodeStatus::FAILURE;
    }

    // Open file stream.
    std::ifstream f(file_path);
    if (!f.is_open()) {
        RCLCPP_ERROR(logger, "[COMMAND] Cannot read mission: file not found at %s", file_path.c_str());
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
                RCLCPP_WARN(logger, "[COMMAND] Unknown logic '%s' - defaulting to GENERIC_OBJECT", logic_str.c_str());
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
            RCLCPP_ERROR(logger, "[COMMAND] Invalid mission: missing cluster_info.cluster_id");
            return BT::NodeStatus::FAILURE;
        }
        setOutput("cluster", cluster_id_value);

        std::string anchor_class = "";
        std::string anchor_id = "";
        if (data.contains("anchor_object_class") && data["anchor_object_class"].is_string()) {
            anchor_class = data["anchor_object_class"].get<std::string>();
        }
        setOutput("anchor_class", anchor_class);
        if (data.contains("anchor_object_id") && data["anchor_object_id"].is_string()) {
            anchor_id = data["anchor_object_id"].get<std::string>();
        }
        setOutput("anchor_id", anchor_id);

        
        RCLCPP_INFO(logger, "[COMMAND] Mission received: action='%s', find '%s' in cluster %d",
                    action.c_str(), goal_class.c_str(), cluster_id_value);

        return BT::NodeStatus::SUCCESS;

    } catch (const json::parse_error& e) {
        RCLCPP_ERROR(logger, "[COMMAND] Mission file is not valid JSON: %s", e.what());
        return BT::NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "[COMMAND] Failed to read mission: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}