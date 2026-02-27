#include "bt_pkg/check_goal_seen.hpp"

BT::NodeStatus CheckGoalSeen::tick()
{
	auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
	if (!node) {
		throw std::runtime_error("Missing ROS Node in Blackboard");
	}

	ensureSubscription(node);

	std::lock_guard<std::mutex> lock(mutex_);
	if (!has_goal_pose_) {
		return BT::NodeStatus::FAILURE;
	}

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
	std::string topic;
	if (!getInput("goal_topic", topic) || topic.empty()) {
		throw BT::RuntimeError("[CheckGoalSeen] missing required input [goal_topic]");
	}

	if (goal_pose_sub_ && node_ == node && subscribed_topic_ == topic) {
		return;
	}

	node_ = node;
	subscribed_topic_ = topic;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		has_goal_pose_ = false;
	}

	auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
	goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		subscribed_topic_,
		qos,
		std::bind(&CheckGoalSeen::onGoalPose, this, std::placeholders::_1));

	RCLCPP_INFO(node_->get_logger(), "[CheckGoalSeen] Subscribed to %s", subscribed_topic_.c_str());
}

void CheckGoalSeen::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mutex_);
	latest_goal_pose_ = *msg;
	has_goal_pose_ = true;
}
