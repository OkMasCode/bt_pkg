#include "bt_pkg/select_goal.hpp"
#include <cmath>
#include <limits>

// Constructor: Setup the direct subscriber to the Semantic Map
SelectGoal::SelectGoal(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }
    
    // Subscribe to the map with Best Effort QoS to match the publisher
    rclcpp::QoS qos(10);
    qos.best_effort();
    qos.durability_volatile();
    
    map_sub_ = node->create_subscription<yolo11_seg_interfaces::msg::SemanticObjectArray>(
        "/vision/semantic_map_v5", qos,
        [this](const yolo11_seg_interfaces::msg::SemanticObjectArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            latest_map_ = msg;
        });
}

BT::PortsList SelectGoal::providedPorts()
{
    return {
        BT::InputPort<LogicType>("logic"),
        BT::InputPort<std::string>("target_class"), // Replaces candidate arrays
        BT::InputPort<geometry_msgs::msg::PoseStamped>("cluster_centroid"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"),
        BT::InputPort<double>("similarity_threshold"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
        BT::OutputPort<bool>("is_object_goal")
    };
}

BT::NodeStatus SelectGoal::tick()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    LogicType logic;
    std::string target_class;
    geometry_msgs::msg::PoseStamped cluster_centroid;
    geometry_msgs::msg::PoseStamped start_pose;
    double similarity_threshold = 15.0; // Default threshold

    if (!getInput("logic", logic)) return BT::NodeStatus::FAILURE;
    if (!getInput("target_class", target_class)) return BT::NodeStatus::FAILURE;
    if (!getInput("cluster_centroid", cluster_centroid)) return BT::NodeStatus::FAILURE;
    if (!getInput("start_pose", start_pose)) return BT::NodeStatus::FAILURE;
    getInput("similarity_threshold", similarity_threshold);

    // Safely copy the latest map from the subscriber callback
    yolo11_seg_interfaces::msg::SemanticObjectArray::SharedPtr current_map;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map = latest_map_;
    }

    // Fallback if the vision node hasn't published yet or map is empty
    if (!current_map || current_map->objects.empty()) {
        RCLCPP_WARN(node->get_logger(), "Map is empty or not received yet. Using cluster centroid.");
        setOutput("target_pose", cluster_centroid);
        setOutput("is_object_goal", false);
        return BT::NodeStatus::SUCCESS;
    }

    // 1. Filter objects by the requested class name
    std::vector<yolo11_seg_interfaces::msg::SemanticObject> matching_objs;
    for (const auto& obj : current_map->objects) {
        if (obj.name == target_class) {
            matching_objs.push_back(obj);
        }
    }

    if (matching_objs.empty()) {
        RCLCPP_WARN(node->get_logger(), "No '%s' objects found in map. Using cluster centroid.", target_class.c_str());
        setOutput("target_pose", cluster_centroid);
        setOutput("is_object_goal", false);
        return BT::NodeStatus::SUCCESS;
    }

    // Helper lambda to construct a PoseStamped from a map object
    auto make_pose = [&](const yolo11_seg_interfaces::msg::SemanticObject& obj) {
        geometry_msgs::msg::PoseStamped p;
        
        // FIXED: Use the 'frame' field from the individual SemanticObject
        p.header.frame_id = obj.frame; 
        p.header.stamp = node->now();
        
        p.pose.position.x = obj.pose_map.x;
        p.pose.position.y = obj.pose_map.y;
        p.pose.position.z = obj.pose_map.z;
        p.pose.orientation.w = 1.0;
        return p;
    };

    // 2. Apply Routing Logic
    if (logic == LogicType::GENERIC_OBJECT) {
        RCLCPP_INFO(node->get_logger(), "Logic: GENERIC_OBJECT - selecting best candidate based on distance from robot.");
        
        size_t best_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < matching_objs.size(); ++i) {
            double dx = matching_objs[i].pose_map.x - start_pose.pose.position.x;
            double dy = matching_objs[i].pose_map.y - start_pose.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist) {
                min_dist = dist;
                best_idx = i;
            }
        }

        RCLCPP_INFO(node->get_logger(), "Closest object found at distance: %.2f", min_dist);
        setOutput("target_pose", make_pose(matching_objs[best_idx]));
        setOutput("is_object_goal", true);

    } 
    else if (logic == LogicType::GENERIC_OBJECT_SPECIFIC_LOCATION) {
        RCLCPP_INFO(node->get_logger(), "Logic: GENERIC_OBJECT_SPECIFIC_LOCATION - selecting best candidate with location bias.");
        
        size_t best_idx = 0;
        double min_dist_to_robot = std::numeric_limits<double>::max();
        bool found_in_cluster = false;
        double cluster_radius = 2.0; // 2 meters tolerance to be considered "inside" the cluster

        for (size_t i = 0; i < matching_objs.size(); ++i) {
            // Check if object is physically near the requested cluster centroid
            double cx = matching_objs[i].pose_map.x - cluster_centroid.pose.position.x;
            double cy = matching_objs[i].pose_map.y - cluster_centroid.pose.position.y;
            
            if (std::hypot(cx, cy) <= cluster_radius) {
                // It is in the cluster! Now find the one closest to the robot.
                double dx = matching_objs[i].pose_map.x - start_pose.pose.position.x;
                double dy = matching_objs[i].pose_map.y - start_pose.pose.position.y;
                double dist = std::hypot(dx, dy);

                if (dist < min_dist_to_robot) {
                    min_dist_to_robot = dist;
                    best_idx = i;
                    found_in_cluster = true;
                }
            }
        }

        if (found_in_cluster) {
            RCLCPP_INFO(node->get_logger(), "Selected closest object in cluster at distance: %.2f", min_dist_to_robot);
            setOutput("target_pose", make_pose(matching_objs[best_idx]));
            setOutput("is_object_goal", true);
        } else {
            RCLCPP_WARN(node->get_logger(), "No '%s' found near cluster centroid. Using centroid.", target_class.c_str());
            setOutput("target_pose", cluster_centroid);
            setOutput("is_object_goal", false);
        }

    } 
    else if (logic == LogicType::SPECIFIC_OBJECT_WITH_FEATURES) {
        RCLCPP_INFO(node->get_logger(), "Logic: SPECIFIC_OBJECT_WITH_FEATURES - selecting best candidate with feature-based scoring.");

        size_t best_idx = 0;
        double max_sim = -1.0;

        for (size_t i = 0; i < matching_objs.size(); ++i) {
            if (matching_objs[i].similarity > max_sim) {
                max_sim = matching_objs[i].similarity;
                best_idx = i;
            }
        }

        if (max_sim >= similarity_threshold) {
            RCLCPP_INFO(node->get_logger(), "Selecting goal pose (score=%.2f >= %.2f) at (%.2f, %.2f)",
                        max_sim, similarity_threshold,
                        matching_objs[best_idx].pose_map.x,
                        matching_objs[best_idx].pose_map.y);
            setOutput("target_pose", make_pose(matching_objs[best_idx]));
            setOutput("is_object_goal", true);
        } else {
            RCLCPP_INFO(node->get_logger(), "Selecting cluster centroid (score=%.2f < %.2f) at (%.2f, %.2f)",
                        max_sim, similarity_threshold,
                        cluster_centroid.pose.position.x,
                        cluster_centroid.pose.position.y);
            setOutput("target_pose", cluster_centroid);
            setOutput("is_object_goal", false);
        }
    } else {
        RCLCPP_WARN(node->get_logger(), "Unknown logic type. Defaulting to GENERIC_OBJECT behavior.");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}