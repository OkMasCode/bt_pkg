#pragma once

#include <mutex>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class CheckGoalSeen : public BT::ConditionNode
{
public:
	CheckGoalSeen(const std::string& name, const BT::NodeConfiguration& config)
			: BT::ConditionNode(name, config)
	{}

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("goal_topic"),
			BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
		};
	}

	BT::NodeStatus tick() override;

private:
	void ensureSubscription(rclcpp::Node::SharedPtr& node);
	void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	rclcpp::Node::SharedPtr node_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	std::string subscribed_topic_;

	std::mutex mutex_;
	geometry_msgs::msg::PoseStamped latest_goal_pose_;
	bool has_goal_pose_{false};
};
