
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "utils.hpp"
#include "move_floor.hpp"


FloorRobotNode::FloorRobotNode()
    : Node("floor_robot"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot")
{
    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);

    // Create callback groups
    // auto service_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Create the service to move the robot
    // move_robot_service_ = create_service<std_srvs::srv::Trigger>(
    //     "/move_floor_robot",
    //     std::bind(&FloorRobotNode::moveRobotCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create a service to pause the motion.
    pause_robot_service_ = create_service<std_srvs::srv::Trigger>(
        "/robot/pause",
        std::bind(&FloorRobotNode::pauseRobotCallback, this, std::placeholders::_1, std::placeholders::_2));

    move_robot_service_ = create_service<rwa5_2::srv::PickPlace>(
        "/robot/move",
        std::bind(&FloorRobotNode::moveRobotCallback, this, std::placeholders::_1, std::placeholders::_2));


    // Create a service to pause the motion.
    // resume_robot_service_ = create_service<std_srvs::srv::Trigger>(
    //     "/robot/resume",
    //     std::bind(&FloorRobotNode::resumeRobotCallback, this, std::placeholders::_1, std::placeholders::_2));


    enable_gripper_client_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
    change_gripper_tool_client_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
    
    // Create a SubscriptionOptions object
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = subscription_callback_group;

    gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state",10,
        std::bind(&FloorRobotNode::floor_gripper_state_cb, this, std::placeholders::_1),
        subscription_options);
}
void FloorRobotNode::moveRobotCallback(
    const rwa5_2::srv::PickPlace::Request::SharedPtr request ,
    rwa5_2::srv::PickPlace::Response::SharedPtr response)
{
    // Define the target pose
    geometry_msgs::msg::Pose target_pose;

    // Set the position
    target_pose.position.x = -2.080; // X coordinate
    target_pose.position.y = 2.805;    // Y coordinate
    target_pose.position.z = 0.723 + utils::OFFSETS["part"];  // Z coordinate

    // Convert roll, pitch, yaw to quaternion
    double roll = -3.14;   // Roll angle in radians
    double pitch = 0.00;   // Pitch angle in radians
    double yaw = -1.57;    // Yaw angle in radians
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
    if (success)
    {
        auto result = floor_robot_.execute(plan);
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            response->success = true;
            response->message = "Robot moved successfully";
            changeGripperState(true);
        }
        else
        {
            response->success = false;
            response->message = "Trajectory execution failed with error code: " + std::to_string(result.val);
        }
    }
    else
    {
        response->message = "Unable to generate trajectory";
        response->success = false;
    }
}

void FloorRobotNode::pauseRobotCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                       std_srvs::srv::Trigger::Response::SharedPtr response){
    
    // Stopping the current execution
    // if pick action
    if (pick_place_==1){
        floor_robot_.stop();
        response->success=true;
        response->message = "Stopped the Action";
    }
    else if(pick_place_==0){
        response->success=true;
        response->message = "No Current Action";
    }
    else{
        response->success=false;
        response->message = "Can't Stop Place Action";
    }
    
}
                    
// void FloorRobotNode::resumeRobotCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
//                        std_srvs::srv::Trigger::Response::SharedPtr response){
    
// }

void FloorRobotNode::floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::SharedPtr msg)
{
    floor_gripper_state_= *msg;
  // This function will be called whenever a message is received on the "/ariac/floor_robot_gripper_state" topic
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received gripper state:");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), floor_gripper_state_.type.c_str());
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Suction enabled: %s", msg->enabled ? "true" : "false");
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object attached: %s", msg->attached ? "true" : "false");
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper type: %s", msg->type.c_str());
}


bool FloorRobotNode::changeGripperState(bool request_en){
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();

    request->enable = request_en;
    auto result_future = enable_gripper_client_->async_send_request(request);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service to get response for change gripper state enabling suction");
    auto response = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got response for change gripper state enabling suction");
    

    if (response->success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully Enabled Gripper");
        return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Not change the state of the gripper");
    return false;
}


bool FloorRobotNode::changeGripperTool(uint8_t gripper_type){
    auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

    request->gripper_type = gripper_type; //ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    auto result_future = change_gripper_tool_client_->async_send_request(request);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service to get response");
    result_future.wait();
    auto response = result_future.get();

    if (response->success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully Change Gripper");
        return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error : %s",response->message.c_str());
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloorRobotNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

