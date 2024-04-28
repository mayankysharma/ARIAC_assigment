#include "move_floor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

FloorRobotNode::FloorRobotNode()
: Node("floor_robot"), floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot")
{

    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);
    // Create the service to move the robot
    move_robot_service_ = create_service<std_srvs::srv::Trigger>(
        "/move_floor_robot",
        std::bind(&FloorRobotNode::moveRobotCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void FloorRobotNode::moveRobotCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
    std_srvs::srv::Trigger::Response::SharedPtr response
) {
    // Define the target pose
  geometry_msgs::msg::Pose target_pose;

    // Set the position
    target_pose.position.x =-1.33; // X coordinate
    target_pose.position.y = 1.651; // Y coordinate
    target_pose.position.z = 1.933; // Z coordinate

    // Convert roll, pitch, yaw to quaternion
    double roll = 1.57; // Roll angle in radians
    double pitch = -0.0000604; // Pitch angle in radians
    double yaw = 0.0000264; // Yaw angle in radians

    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);

    // Set the orientation
    target_pose.orientation.x = orientation.x();
    target_pose.orientation.y = orientation.y();
    target_pose.orientation.z = orientation.z();
    target_pose.orientation.w = orientation.w();


    // Set the target pose for the robot
    floor_robot_.setPoseTarget(target_pose);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = static_cast<bool>(floor_robot_.plan(plan));
    
    // Execute the plan
    if (success) {
        auto result = floor_robot_.execute(plan);
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            response->success = true;
            response->message = "Robot moved successfully";
        } else {
            response->success = false;
            response->message = "Trajectory execution failed with error code: " + std::to_string(result.val);
        }
    } else {
        response->message = "Unable to generate trajectory";
        response->success = false;
    }
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloorRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
