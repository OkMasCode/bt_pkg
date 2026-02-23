#include "bt_pkg/find_approach_pose.hpp"

#include <cmath>

void FindApproachPose::ensureSubscription(const rclcpp::Node::SharedPtr& node, const std::string& topic)
{
    // Reuse existing subscription when topic has not changed.
    if (costmap_sub_ && topic == last_topic_) {
        return;
    }

    // (Re)subscribe and cache the latest costmap for future ticks.
    last_topic_ = topic;
    costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic, rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            last_costmap_ = std::move(msg);
        });
}

bool FindApproachPose::isFree(const nav_msgs::msg::OccupancyGrid& grid, double wx, double wy, int free_threshold) const
{
    // Convert world coordinates into costmap index space.
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    const double resolution = grid.info.resolution;

    if (resolution <= 0.0) {
        return false;
    }

    const int mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    const int my = static_cast<int>(std::floor((wy - origin_y) / resolution));

    if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.info.width) || my >= static_cast<int>(grid.info.height)) {
        return false;
    }

    const int idx = my * static_cast<int>(grid.info.width) + mx;
    if (idx < 0 || idx >= static_cast<int>(grid.data.size())) {
        return false;
    }

    // Unknown cells are treated as not free for safety.
    const int8_t occ = grid.data[idx];
    if (occ < 0) {
        return false; // unknown treated as not free
    }

    return occ <= free_threshold;
}

BT::NodeStatus FindApproachPose::tick()
{
    // Retrieve ROS node from blackboard for logging/subscriptions/time.
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }

    // Required target pose input.
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("goal_pose", goal_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Missing goal_pose input");
        return BT::NodeStatus::FAILURE;
    }

    // Optional parameters with defaults.
    std::string costmap_topic = "/global_costmap/costmap";
    getInput("costmap_topic", costmap_topic);

    double min_radius = 0.5;
    double max_radius = 1.5;
    double radius_step = 0.1;
    double angle_step_deg = 30.0;
    int free_threshold = 0;

    getInput("min_radius", min_radius);
    getInput("max_radius", max_radius);
    getInput("radius_step", radius_step);
    getInput("angle_step_deg", angle_step_deg);
    getInput("free_threshold", free_threshold);

    // Validate sampling configuration before planning candidates.
    if (min_radius < 0.0 || max_radius < min_radius || radius_step <= 0.0 || angle_step_deg <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Invalid sampling parameters");
        return BT::NodeStatus::FAILURE;
    }

    ensureSubscription(node, costmap_topic);

    // Wait until at least one costmap has been received.
    if (!last_costmap_) {
        RCLCPP_WARN(node->get_logger(), "No costmap received yet on %s", costmap_topic.c_str());
        return BT::NodeStatus::FAILURE;
    }

    const auto& grid = *last_costmap_;
    const double goal_x = goal_pose.pose.position.x;
    const double goal_y = goal_pose.pose.position.y;

    geometry_msgs::msg::PoseStamped approach_pose;
    bool found = false;

    // Search concentric rings around the goal and pick the first free sampled point.
    for (double r = min_radius; r <= max_radius && !found; r += radius_step) {
        for (double a = 0.0; a < 360.0; a += angle_step_deg) {
            const double rad = a * M_PI / 180.0;
            const double cx = goal_x + r * std::cos(rad);
            const double cy = goal_y + r * std::sin(rad);

            if (!isFree(grid, cx, cy, free_threshold)) {
                continue;
            }

            approach_pose.header.frame_id = !grid.header.frame_id.empty() ? grid.header.frame_id : goal_pose.header.frame_id;
            approach_pose.header.stamp = node->now();
            approach_pose.pose.position.x = cx;
            approach_pose.pose.position.y = cy;
            approach_pose.pose.position.z = goal_pose.pose.position.z;

            // Orient approach pose to face the goal.
            const double yaw = std::atan2(goal_y - cy, goal_x - cx);
            approach_pose.pose.orientation.x = 0.0;
            approach_pose.pose.orientation.y = 0.0;
            approach_pose.pose.orientation.z = std::sin(yaw * 0.5);
            approach_pose.pose.orientation.w = std::cos(yaw * 0.5);

            found = true;
            break;
        }
    }

    if (!found) {
        RCLCPP_WARN(node->get_logger(), "No free approach pose found around goal");
        return BT::NodeStatus::FAILURE;
    }

    // Publish selected approach pose to the BT blackboard.
    setOutput("approach_pose", approach_pose);
    return BT::NodeStatus::SUCCESS;
}
