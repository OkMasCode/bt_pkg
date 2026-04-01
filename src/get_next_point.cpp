#include "bt_pkg/get_next_point.hpp"
#include <random>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

GetNextPoint::GetNextPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetNextPoint::tick()
{
    geometry_msgs::msg::PoseStamped centroid;
    std::vector<double> dimensions;

    // 1. Read the center and size of the room/cluster
    if (!getInput("cluster_centroid", centroid)) {
        throw BT::RuntimeError("Missing required input [cluster_centroid]");
    }
    if (!getInput("cluster_dimensions", dimensions) || dimensions.size() < 2) {
        throw BT::RuntimeError("Missing or invalid input [cluster_dimensions]. Expected [length_x, width_y].");
    }

    // 2. Define the boundaries of the cluster (Length / 2)
    double half_x = dimensions[0] / 2.0;
    double half_y = dimensions[1] / 2.0;

    // 3. Setup Random Number Generator
    // We use a random_device to seed the Mersenne Twister engine for high-quality randomness
    std::random_device rd;  
    std::mt19937 gen(rd()); 
    
    // Create distributions bounded by the size of the cluster
    std::uniform_real_distribution<double> dist_x(-half_x, half_x);
    std::uniform_real_distribution<double> dist_y(-half_y, half_y);
    std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI); // Random facing direction

    // Generate random offsets from the center of the cluster
    double dx = dist_x(gen);
    double dy = dist_y(gen);

    // 4. Construct the next exploration waypoint
    geometry_msgs::msg::PoseStamped next_point;
    next_point.header = centroid.header; 
    
    // Apply the offset to the centroid to get a point inside the cluster
    next_point.pose.position.x = centroid.pose.position.x + dx;
    next_point.pose.position.y = centroid.pose.position.y + dy;
    next_point.pose.position.z = centroid.pose.position.z;

    // 5. Apply a random orientation 
    // Since the robot will do a 360 spin anyway, initial orientation isn't critical,
    // but randomizing it prevents the local planner from getting stuck in repetitive motion patterns.
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dist_yaw(gen));
    next_point.pose.orientation.x = q.x();
    next_point.pose.orientation.y = q.y();
    next_point.pose.orientation.z = q.z();
    next_point.pose.orientation.w = q.w();

    // 6. Write the point to the blackboard
    setOutput("next_point", next_point);

    return BT::NodeStatus::SUCCESS;
}