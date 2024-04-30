#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
//Ariac msgs
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
//Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
//
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.h>
// TF2
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// KDL
#include <tf2_kdl/tf2_kdl.h>

#include <kdl/frames.hpp>
//
#include <unistd.h>
#include <cmath>

#include "rwa5_2/srv/pick_place.hpp"
#include "utils.hpp"

using PickPlaceSrv = rwa5_2::srv::PickPlace;
using ChangeGripperSrv = ariac_msgs::srv::ChangeGripper;
using VacuumGripperControlSrv = ariac_msgs::srv::VacuumGripperControl;


class FloorRobotNode : public rclcpp::Node {
public:
    FloorRobotNode();


private:
 // Move group interface for the floor robo
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    //! Planning scene interface for the workcell
    moveit::planning_interface::PlanningSceneInterface planning_scene_;
  //! Trajectory processing for the floor robot
  /*!
  Generate the time-optimal trajectory along a given path within given bounds
  on accelerations and velocities.
  */
    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

    // Subscriber
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr gripper_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr go_home_sub_;
    //Callback group for gripper
    rclcpp::CallbackGroup::SharedPtr gripper_cbg_;


    // Service clients
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr change_gripper_tool_client_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr enable_gripper_client_;
    //! Client for "/ariac/perform_quality_check" service
    rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
    //! Client for "/ariac/floor_robot_change_gripper" service
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;


    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_robot_service_;
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_robot_service_;
    rclcpp::Service<rwa5_2::srv::PickPlace>::SharedPtr move_robot_service_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_robot_service_;

    // Is it plan for pick and place
    // pick 1, place 2
    uint8_t pick_place_ = 0; 
    // bool place_ = false;

    // Check if the service gripper started ?
    bool _change_gripper_tool_service_started = false;
    // get the gripper service value to add in change gripper tool
    uint8_t _change_gripper_tool_service_value = 0;


    // Check if the service gripper started ?
    bool _enable_gripper_service_started = false;
    // get the gripper service value to add in change gripper state vacuum enabled
    bool _enable_gripper_service_value = false;

    // Timer 
    rclcpp::TimerBase::SharedPtr timer_;

    void floor_robot_sub_cb(const std_msgs::msg::String::ConstSharedPtr msg);
    // Callback function for gripper state subscriber


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


//     /**
//      * @brief Using the service enable the gripper 
//      * service : "/ariac/floor_robot_enable_gripper"
//      */
//     bool changeGripperState(rclcpp::Client<VacuumGripperControlSrv>::SharedFuture future);

//     /**
//      * @brief Change the gripper tool given from gripper client
//      * service : "/ariac/floor_robot_change_gripper"
//      */
//     bool changeGripperTool(rclcpp::Client<ChangeGripperSrv>::SharedFuture future);


    bool move_to_target ();
    /**
     * @brief Using the service enable the gripper 
     * service : "/ariac/floor_robot_enable_gripper"
     */
    bool changeGripperState(bool request);

    /**
     * @brief Change the gripper tool given from gripper client
     * service : "/ariac/floor_robot_change_gripper"
     */
    bool changeGripperTool(uint8_t request);
    /**
     * move_through_waypoints function
     * @brief movve through waypoints
     *  @param waypoints  Waypoints to move through
     * @param vsf  Velocity scale factor
     * @param asf  Acceleration scale factor
     * @return true  Successfully moved through the waypoints
     * @return false  Failed to move through the waypoints
    */
    bool move_through_waypoints (std::vector<geometry_msgs::msg::Pose> waypoints,
                                double vsf, double asf);
    /**
     * @brief  Move the robot to its home pose
     *
     * @return true Motion successful
     * @return false Motion failed
     */


    /**
     * @brief Add a single model to the planning scene
     *
     * @param name  Name of the model
     * @param mesh_file  Mesh file of the model
     * @param model_pose  Pose of the model
     */
    void
    add_single_model_to_planning_scene (std::string name, std::string mesh_file,
                                        geometry_msgs::msg::Pose model_pose);
    //-----------------------------//
    /**
     * @brief Add static models to the planning scene
     *
     * Static models include the bins, tray tables, assembly stations, assembly
     * inserts, and the conveyor belt
     *
     */

    void add_models_to_planning_scene ();
    //-----------------------------//
    void wait_for_attach_completion (double timeout);

    /**
     * @brief move the floor robot to gripper change station to change the gripper
     * 
     * @param target_pose, pose of the gripper station
     */
    void moveToGripperStation(geometry_msgs::msg::Pose target_pose, int depth = 1, float offset = utils::OFFSETS["tool_loc"]);

    /**
     * @brief run the service according flag, and check after every duration 
     * if need to run the service or not.
     * 
     */
    void serviceTimerCallback();

};