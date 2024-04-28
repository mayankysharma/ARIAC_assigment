#include "move_floor.hpp"

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
)
{
    // Define the target pose
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.position.x = 2.0;  // Set the X coordinate
    // target_pose.position.y = 3.0;   // Keep the Y coordinate as it is
    // target_pose.position.z = 3.0;   // Keep the Z coordinate as it is
    // target_pose.orientation.w = 1.0;  // Default orientation
    auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
    }();
    // Set the target pose for the robot
    floor_robot_.setPoseTarget(target_pose);
    auto const [success, plan] = [&floor_robot_]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(floor_robot_.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  floor_robot.execute(plan);
} else {
  RCLCPP_ERROR(this->get_logger(), "Planning failed!");
}
}
//     RCLCPP_INFO(this->get_logger(), "Active Joint list"); // Convert std::string to C-style string

//     for (auto aj: floor_robot_.getActiveJoints()){
//         RCLCPP_INFO(this->get_logger(), "%s", aj.c_str()); // Convert std::string to C-style string
//     }


//     RCLCPP_INFO(this->get_logger(), "======="); // Convert std::string to C-style string
//     RCLCPP_INFO(this->get_logger(), "Link list"); // Convert std::string to C-style string

//     for (auto aj: floor_robot_.getLinkNames()){
//         RCLCPP_INFO(this->get_logger(), "%s", aj.c_str()); // Convert std::string to C-style string
//     }

//     // // Plan the trajectory
//     // moveit::planning_interface::MoveGroupInterface::Plan plan;
//     // std::string end_effector_link = floor_robot_.getEndEffectorLink();
//     // RCLCPP_INFO(this->get_logger(), "%s", end_effector_link.c_str()); // Convert std::string to C-style string

//     // bool success = static_cast<bool>(floor_robot_.plan(plan));

//     if (success) {
//         // Execute the planned trajectory
//         if (static_cast<bool>(floor_robot_.execute(plan))) {
//             response->success = true;
//             response->message = "Robot moved successfully";
//         } else {
//             response->success = false;
//             response->message = "Trajectory execution failed";
//         }
//     } else {
//         response->message = "Unable to generate trajectory";
//         response->success = false;
//     }
// }

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloorRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
