
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "utils.hpp"
#include "move_floor.hpp"


FloorRobotNode::FloorRobotNode()
    : Node("floor_robot"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot")
{
    floor_robot_.setMaxAccelerationScalingFactor(0.5);
    floor_robot_.setMaxVelocityScalingFactor(0.7);

    // allow replanning in dynamic environment
    floor_robot_.allowReplanning(true);

    // Set the number of times the motion plan is to be computed from scratch before the shortest solution is returned. The default value is 1. 
    floor_robot_.setNumPlanningAttempts(5);
    floor_robot_.setPlanningTime(5.0); // Set the maximum planning time to 1 seconds

    // Create callback groups
    auto move_robot_service_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto gripper_service_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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

    enable_gripper_client_ = this->create_client<VacuumGripperControlSrv>("/ariac/floor_robot_enable_gripper",rmw_qos_profile_services_default, gripper_service_group);
    change_gripper_tool_client_ = this->create_client<ChangeGripperSrv>("/ariac/floor_robot_change_gripper",rmw_qos_profile_services_default, gripper_service_group);
    
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

    float offset = 0.0;
    if (request->part_type!=-1)
        offset = utils::OFFSETS["part"] + utils::PART_HEIGHTS[request->part_type];
    else
    {
        offset = utils::OFFSETS["tray"]; //+ utils::PART_HEIGHTS[request->tray_id];
    } 

    // Set the position
    target_pose.position.x = request->destination_pose.position.x; // X coordinate
    target_pose.position.y = request->destination_pose.position.y;    // Y coordinate
    target_pose.position.z = request->destination_pose.position.z + offset;  // Z coordinate

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Target pose x: %f, y: %f, z: %f",target_pose.position.x,target_pose.position.y,target_pose.position.z);
       
    // Convert roll, pitch, yaw to quaternion
    double roll = 3.14;   // Roll angle in radians
    double pitch = 0.00;   // Pitch angle in radians
    double yaw = 1.57;    // Yaw angle in radians
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

            _enable_gripper_service_started = true;
            auto request_vacuum_gripper = std::make_shared<VacuumGripperControlSrv::Request>();
            if (request->pick_place==PickPlaceSrv::Request::PICK) request_vacuum_gripper->enable = true;
            else request_vacuum_gripper->enable = false;

            enable_gripper_client_->async_send_request(request_vacuum_gripper,std::bind(&FloorRobotNode::changeGripperState, this, std::placeholders::_1));
            while (_enable_gripper_service_started){continue;}

            _change_gripper_tool_service_started = true;
            auto request_change_gripper = std::make_shared<ChangeGripperSrv::Request>();
            if (request->tray_id==-1) request_change_gripper->gripper_type = ChangeGripperSrv::Request::PART_GRIPPER; //ariac_msg s::srv::ChangeGripper::Request::PART_GRIPPER;
            else request_change_gripper->gripper_type = ChangeGripperSrv::Request::TRAY_GRIPPER;
            change_gripper_tool_client_->async_send_request(request_change_gripper,std::bind(&FloorRobotNode::changeGripperTool, this, std::placeholders::_1));            
            while (_change_gripper_tool_service_started ){continue;}
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
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received gripper state:");
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), floor_gripper_state_.type.c_str());
  
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Suction enabled: %s", msg->enabled ? "true" : "false");
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object attached: %s", msg->attached ? "true" : "false");
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper type: %s", msg->type.c_str());
}


bool FloorRobotNode::changeGripperState(rclcpp::Client<VacuumGripperControlSrv>::SharedFuture future){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service to get response for change gripper state enabling suction");
    auto response = future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got response for change gripper state enabling suction");
    

    if (response->success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully Enabled Gripper");
        // return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Not change the state of the gripper");
    _enable_gripper_service_started = false;
}


bool FloorRobotNode::changeGripperTool(rclcpp::Client<ChangeGripperSrv>::SharedFuture future){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service to get response");
    // result_future.wait();
    auto response = future.get();

    if (response->success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully Change Gripper");
        // return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error : %s",response->message.c_str());
    _change_gripper_tool_service_started = false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloorRobotNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); 
    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

