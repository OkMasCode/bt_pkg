#ifndef BT_PKG__GET_NEXT_POINT_HPP_
#define BT_PKG__GET_NEXT_POINT_HPP_

#include "behaviortree_cpp/action_node.h" // Note: use behaviortree_cpp_v3/action_node.h if on older BT.CPP
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

class GetNextPoint : public BT::SyncActionNode
{
public:
    // Change BT::NodeConfiguration to BT::NodeConfig
    GetNextPoint(const std::string& name, const BT::NodeConfig& config);

    // 1. Update the ports to match the new architecture
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("target_cluster", "The ID of the room to explore"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("next_point", "The generated safe waypoint from Python")
        };
    }

    BT::NodeStatus tick() override;

private:
    // 2. Add the ROS 2 node pointer so we can create service clients
    rclcpp::Node::SharedPtr node_;
};

#endif // BT_PKG__GET_NEXT_POINT_HPP_