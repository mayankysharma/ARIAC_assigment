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
class FloorRobotNode : public rclcpp::Node {
public:
    FloorRobotNode();

private:
    moveit::planning_interface::MoveGroupInterface floor_robot_;

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_robot_service_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_gripper_service_;
ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
// Subscriber
rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr gripper_state_sub_;

// Callback function for gripper state subscriber
void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::SharedPtr msg);

// Service clients
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr move_robot_client_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_gripper_client_;

// Callback function for move robot service
void moveRobotCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                       std_srvs::srv::Trigger::Response::SharedPtr response);


void enableGripperService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                       std_srvs::srv::Trigger::Response::SharedPtr response);

};