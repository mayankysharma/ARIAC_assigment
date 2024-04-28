#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class FloorRobotNode : public rclcpp::Node {
public:
    FloorRobotNode();
   
private:
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_robot_service_;

    void moveRobotCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response
    );
};
