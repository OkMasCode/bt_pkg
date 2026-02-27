#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

// 1. INCLUDE YOUR LEAF HEADER
// (Ensure this file exists in include/my_robot_behavior/)
#include "bt_pkg/select_goal.hpp"
#include "bt_pkg/read_json.hpp"
#include "bt_pkg/navigate_to_pose.hpp"
#include "bt_pkg/find_approach_pose.hpp"
#include "bt_pkg/check_is_object.hpp"
#include "bt_pkg/check_action.hpp"
#include "bt_pkg/rotate_360.hpp"
#include "bt_pkg/get_robot_start_pose.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS 2 runtime.
    rclcpp::init(argc, argv);
    
    // Shared node used by BT leaf nodes for ROS APIs (actions, pubs/subs, TF, etc.).
    auto node = std::make_shared<rclcpp::Node>("bt_manager");

    // Create BehaviorTree.CPP factory and register all custom leaf nodes.
    BT::BehaviorTreeFactory factory;

    // Tag names must match node IDs used in the XML tree.
    factory.registerNodeType<ReadJson>("ReadJson");
    factory.registerNodeType<SelectGoal>("SelectGoal");
    factory.registerNodeType<NavigateToPose>("NavigateToPose");
    factory.registerNodeType<FindApproachPose>("FindApproachPose");
    factory.registerNodeType<CheckIsObject>("CheckIsObject");
    factory.registerNodeType<CheckAction>("CheckAction");
    factory.registerNodeType<Rotate360>("Rotate360");
    factory.registerNodeType<GetRobotStartPose>("GetRobotStartPose");

    // Configure blackboard entries shared by all BT nodes.
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);

    // Absolute path to BT XML definition.
    std::string xml_path = "/workspaces/ros2_ws/src/bt_pkg/bt_xml/behavior_tree.xml";
    
    try {
        // Instantiate tree with the configured blackboard.
        auto tree = factory.createTreeFromFile(xml_path, blackboard);

        // Tick loop at 10 Hz until success, failure retry, or ROS shutdown.
        rclcpp::Rate rate(10);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        
        while (rclcpp::ok())
        {
            status = tree.tickOnce();
            
            if (status == BT::NodeStatus::SUCCESS) {
                // Mission complete.
                RCLCPP_INFO(node->get_logger(), "Behavior Tree completed successfully!");
                break;
            } else if (status == BT::NodeStatus::FAILURE) {
                // Keep ticking to allow recovery/fallback branches on next cycle.
                RCLCPP_WARN(node->get_logger(), "Behavior Tree failed, retrying...");
            }
            
            // Process ROS callbacks required by asynchronous BT nodes.
            rclcpp::spin_some(node);
            rate.sleep();
        }
    }
    catch (const std::exception& ex) {
        // Covers XML load/parse errors and runtime tree construction issues.
        RCLCPP_ERROR(node->get_logger(), "Tree Error: %s", ex.what());
    }

    // Clean shutdown.
    rclcpp::shutdown();
    return 0;
}