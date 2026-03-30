    #include "bt_pkg/select_goal.hpp"

    BT::NodeStatus SelectGoal::tick()
    {
        // Retrieve ROS node handle from the BT blackboard for logging and runtime context.
        auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node) {
            throw std::runtime_error("Missing ROS Node in Blackboard");
        }
        
        std::vector<std::string> candidates_ids;
        std::vector<double> similarity_scores;
        std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
        std::vector<int> cluster_ids;
        int cluster;
        geometry_msgs::msg::PoseStamped cluster_centroid;
        geometry_msgs::msg::PoseStamped start_pose;
        double similarity_threshold = 8.0;
        LogicType logic;

        if (!getInput("logic", logic)) {
            RCLCPP_ERROR(node->get_logger(), "Missing logic input");
            return BT::NodeStatus::FAILURE;
        }
        // Read required inputs that describe candidate object matches and corresponding poses.
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
        if (!getInput("cluster_ids", cluster_ids)) {
            RCLCPP_ERROR(node->get_logger(), "Missing cluster_ids input");
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput("cluster", cluster)) {
            RCLCPP_ERROR(node->get_logger(), "Missing cluster input");
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput("start_pose", start_pose)) {
            RCLCPP_ERROR(node->get_logger(), "Missing start_pose input");
            return BT::NodeStatus::FAILURE;
        }
        // Optional threshold: defaults to 8.0 if not provided.
        getInput("similarity_threshold", similarity_threshold);

        // If perception outputs are empty, fall back to exploring the cluster centroid.
        if (candidates_ids.empty() || similarity_scores.empty() || goal_poses.empty()) {
            RCLCPP_WARN(node->get_logger(), "Empty candidates/similarity/poses. Using cluster centroid.");
            setOutput("target_pose", cluster_centroid);
            return BT::NodeStatus::SUCCESS;
        }

            // Guard against inconsistent vectors; otherwise indexing could be unsafe.
        if (similarity_scores.size() != goal_poses.size()) {
            RCLCPP_WARN(node->get_logger(), "Mismatch sizes: similarity_scores=%zu, goal_poses=%zu. Using cluster centroid.",
                        similarity_scores.size(), goal_poses.size());
            setOutput("target_pose", cluster_centroid);
            return BT::NodeStatus::SUCCESS;
        }

        if (logic == LogicType::GENERIC_OBJECT) {
            RCLCPP_INFO(node->get_logger(), "Logic: GENERIC_OBJECT - selecting best candidate based on distance from robot.");
            
            size_t closest_idx = 0;
            double min_dist = std::numeric_limits<double>::max();

            for (size_t i = 0; i < goal_poses.size(); ++i) {
                // Calculate Euclidean distance (dx^2 + dy^2)
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
            RCLCPP_INFO(node->get_logger(), "Logic: GENERIC_OBJECT_SPECIFIC_LOCATION - selecting best candidate with location bias.");
            
            std::vector<size_t> in_cluster_indices = {};
            
            for (size_t i = 0; i < goal_poses.size(); ++i) {
                // Ensure we don't go out of bounds if cluster_ids is smaller than goal_poses
                if (i < cluster_ids.size() && cluster_ids[i] == cluster) {
                    in_cluster_indices.push_back(i);
                }
            }

            if (in_cluster_indices.empty()) {
                // FIXED: Changed %s to %d and removed .c_str()
                RCLCPP_WARN(node->get_logger(), "No objects found in cluster %d. Using centroid.", cluster);
                setOutput("target_pose", cluster_centroid);
                setOutput("is_object_goal", false);
            } else {
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
            }
            return BT::NodeStatus::SUCCESS;

        } else if (logic == LogicType::SPECIFIC_OBJECT_WITH_FEATURES) {
            RCLCPP_INFO(node->get_logger(), "Logic: SPECIFIC_OBJECT_WITH_FEATURES - selecting best candidate with feature-based scoring.");

            // Select the highest-similarity candidate.
            size_t best_idx = 0;
            double best_score = similarity_scores[0];
            for (size_t i = 1; i < similarity_scores.size(); ++i) {
                if (similarity_scores[i] > best_score) {
                    best_score = similarity_scores[i];
                    best_idx = i;
                }
            }
            // Use object pose only when confidence is above threshold; otherwise use centroid.
            if (best_score >= similarity_threshold) {
                RCLCPP_INFO(node->get_logger(), "Selecting goal pose (score=%.2f >= %.2f) at (%.2f, %.2f)",
                            best_score, similarity_threshold,
                            goal_poses[best_idx].pose.position.x,
                            goal_poses[best_idx].pose.position.y);
                setOutput("target_pose", goal_poses[best_idx]);
                setOutput("is_object_goal", true);
            } else {
                RCLCPP_INFO(node->get_logger(), "Selecting cluster centroid (score=%.2f < %.2f) at (%.2f, %.2f)",
                            best_score, similarity_threshold,
                            cluster_centroid.pose.position.x,
                            cluster_centroid.pose.position.y);
                setOutput("target_pose", cluster_centroid);
                setOutput("is_object_goal", false);
            }
            return BT::NodeStatus::SUCCESS;
            
        } else {
            RCLCPP_WARN(node->get_logger(), "Unknown logic type. Defaulting to GENERIC_OBJECT behavior.");
        }

    }