#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yolo11_seg_interfaces/srv/check_candidates.hpp"

// Shorten the namespace for cleaner code
using CheckCandidates = yolo11_seg_interfaces::srv::CheckCandidates;

class CallCheckCandidates : public BT::StatefulActionNode
{
public:
    CallCheckCandidates(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    // Define the inputs (from JSON) and outputs (to Navigation)
    static BT::PortsList providedPorts()
    {
        return {
            // INPUTS: defined in ReadJson and passed here via XML
            BT::InputPort<std::vector<std::string>>("candidate_ids"),
            
            // OUTPUT: The target for NavigateToPose
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Client<CheckCandidates>::SharedPtr client_;
    
    // This holds the "future" result while we wait for the server
    std::shared_future<std::shared_ptr<CheckCandidates::Response>> future_result_;
};