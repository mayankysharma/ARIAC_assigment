#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unistd.h>
#include <cmath>

#include "rwa5_2/srv/pick_place.hpp"

using PickPlaceSrv = rwa5_2::srv::PickPlace;
using ChangeGripperSrv = ariac_msgs::srv::ChangeGripper;
using VacuumGripperControlSrv = ariac_msgs::srv::VacuumGripperControl;

class FloorRobotNode : public rclcpp::Node {
public:
    FloorRobotNode();

    // Callback function for move robot service
    void moveRobotCallback(
    const rwa5_2::srv::PickPlace::Request::SharedPtr request ,
    rwa5_2::srv::PickPlace::Response::SharedPtr response);
    
    /**
     * @brief Pause the robot plan execution if it is running in pick mode
     * 
     * @param request 
     * @param response 
     */
    void pauseRobotCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);
    
    // /**
    //  * @brief 
    //  * 
    //  * @param request 
    //  * @param response 
    //  */
    // void resumeRobotCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
    //                     std_srvs::srv::Trigger::Response::SharedPtr response);

    // Callback function for gripper state subscriber
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::SharedPtr msg);


    /**
     * @brief Using the service enable the gripper 
     * service : "/ariac/floor_robot_enable_gripper"
     */
    bool changeGripperState(rclcpp::Client<VacuumGripperControlSrv>::SharedFuture future);

    /**
     * @brief Change the gripper tool given from gripper client
     * service : "/ariac/floor_robot_change_gripper"
     */
    bool changeGripperTool(rclcpp::Client<ChangeGripperSrv>::SharedFuture future);

private:
    moveit::planning_interface::MoveGroupInterface floor_robot_;

    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_robot_service_;
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_robot_service_;
    rclcpp::Service<rwa5_2::srv::PickPlace>::SharedPtr move_robot_service_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_robot_service_;

    // Subscriber
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr gripper_state_sub_;

    // Service clients
    rclcpp::Client<ChangeGripperSrv>::SharedPtr change_gripper_tool_client_;
    rclcpp::Client<VacuumGripperControlSrv>::SharedPtr enable_gripper_client_;

    // Is it plan for pick and place
    // pick 1, place 2
    uint8_t pick_place_ = 0; 
    // bool place_ = false;

    bool _change_gripper_tool_service_started = false; 
    bool _enable_gripper_service_started = false;
};