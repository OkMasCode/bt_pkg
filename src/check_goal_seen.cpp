#include "bt_pkg/check_goal_seen.hpp"
// Make sure to add this include in your header file if it's not there:
// #include <std_msgs/msg/bool.hpp>

BT::NodeStatus CheckGoalSeen::tick()
{
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
        throw std::runtime_error("Missing ROS Node in Blackboard");
    }

    ensureSubscription(node);

    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if the goal is currently visible based on the boolean flag
    if (!is_goal_seen_ || !has_goal_pose_) {
        return BT::NodeStatus::FAILURE;
    }

    // Goal is seen, write the latest pose to the blackboard
    if (!setOutput("goal_pose", latest_goal_pose_)) {
        RCLCPP_ERROR(node_->get_logger(), "[CheckGoalSeen] Failed to write 'goal_pose' output port");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "[CheckGoalSeen] Goal pose available: x=%.3f, y=%.3f, z=%.3f",
        latest_goal_pose_.pose.position.x,
        latest_goal_pose_.pose.position.y,
        latest_goal_pose_.pose.position.z);

    return BT::NodeStatus::SUCCESS;
}

void CheckGoalSeen::ensureSubscription(rclcpp::Node::SharedPtr& node)
{
    std::string pose_topic;
    std::string flag_topic;

    // Retrieve both topic names from the BT XML
    if (!getInput("goal_pose_topic", pose_topic) || pose_topic.empty()) {
        throw BT::RuntimeError("[CheckGoalSeen] missing required input [goal_pose_topic]");
    }
    if (!getInput("goal_flag_topic", flag_topic) || flag_topic.empty()) {
        throw BT::RuntimeError("[CheckGoalSeen] missing required input [goal_flag_topic]");
    }

    // Skip initialization if we are already subscribed to the correct topics
    if (goal_pose_sub_ && goal_flag_sub_ && node_ == node && 
        subscribed_pose_topic_ == pose_topic && subscribed_flag_topic_ == flag_topic) {
        return;
    }

    node_ = node;
    subscribed_pose_topic_ = pose_topic;
    subscribed_flag_topic_ = flag_topic;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        has_goal_pose_ = false;
        is_goal_seen_ = false; // Initialize the new flag tracking variable
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    
    // Subscribe to the Pose topic
    goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        subscribed_pose_topic_,
        qos,
        std::bind(&CheckGoalSeen::onGoalPose, this, std::placeholders::_1));

    // Subscribe to the Boolean flag topic
    goal_flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        subscribed_flag_topic_,
        qos,
        std::bind(&CheckGoalSeen::onGoalFlag, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "[CheckGoalSeen] Subscribed to poses on %s and flags on %s", 
                subscribed_pose_topic_.c_str(), subscribed_flag_topic_.c_str());
}

void CheckGoalSeen::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_goal_pose_ = *msg;
    has_goal_pose_ = true;
}

void CheckGoalSeen::onGoalFlag(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    is_goal_seen_ = msg->data; // Updates in real-time if the object is lost or found
}