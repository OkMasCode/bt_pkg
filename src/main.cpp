#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

// 1. INCLUDE YOUR LEAF HEADER
// (Ensure this file exists in include/my_robot_behavior/)
#include "bt_pkg/check_goal_seen.hpp"
#include "bt_pkg/read_json.hpp"
#include "bt_pkg/navigate_to_pose.hpp"
#include "bt_pkg/find_approach_pose.hpp"
#include "bt_pkg/check_is_object.hpp"
#include "bt_pkg/check_action.hpp"
#include "bt_pkg/rotate_360.hpp"
#include "bt_pkg/get_robot_start_pose.hpp"

int main(int argc, char **argv)
{
    // A. Initialize ROS 2 (Standard)
    rclcpp::init(argc, argv);
    
    // Create the ROS node. 
    // This node will handle the communication for the Service Client.
    auto node = std::make_shared<rclcpp::Node>("bt_manager");

    // B. Initialize the Factory
    BT::BehaviorTreeFactory factory;

    // 2. REGISTER YOUR LEAF
    // The string "CallCheckCandidates" MUST match the tag name in your XML file.
    factory.registerNodeType<ReadJson>("ReadJson");
    factory.registerNodeType<CallCheckCandidates>("CallCheckCandidates");
    factory.registerNodeType<NavigateToPose>("NavigateToPose");
    factory.registerNodeType<FindApproachPose>("FindApproachPose");
    factory.registerNodeType<CheckIsObject>("CheckIsObject");
    factory.registerNodeType<CheckAction>("CheckAction");
    factory.registerNodeType<Rotate360>("Rotate360");
    factory.registerNodeType<GetRobotStartPose>("GetRobotStartPose");
    // 3. SETUP BLACKBOARD (Crucial for Service Clients)
    // Your CallCheckCandidates needs a ROS node to create_client().
    // We pass it via the Blackboard.
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);

    // C. Load and Run the Tree
    std::string xml_path = "/workspaces/ros2_ws/src/bt_pkg/bt_xml/behavior_tree.xml";
    
    try {
        // Create the tree using the blackboard we just configured
        auto tree = factory.createTreeFromFile(xml_path, blackboard);

        // Simple Tick Loop (10Hz)
        rclcpp::Rate rate(10);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        
        while (rclcpp::ok())
        {
            status = tree.tickOnce();
            
            if (status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Behavior Tree completed successfully!");
                break;
            } else if (status == BT::NodeStatus::FAILURE) {
                RCLCPP_WARN(node->get_logger(), "Behavior Tree failed, retrying...");
                // Tree will be ticked again on next iteration
            }
            
            rclcpp::spin_some(node); // Handle ROS callbacks (replies from service)
            rate.sleep();
        }
    }
    catch (const std::exception& ex) {
        RCLCPP_ERROR(node->get_logger(), "Tree Error: %s", ex.what());
    }

    rclcpp::shutdown();
    return 0;
}