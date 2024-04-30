
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <iostream>
#include <stdexcept>

#include "utils.hpp"
#include "move_floor.hpp"


FloorRobotNode::FloorRobotNode()
    : Node("floor_robot"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"), planning_scene_()
{

    // auto sim_time = std::make_shared<rclcpp::Parameter::Parameter<bool>>("use_sim_time", true);
    // std::vector<rclcpp::parameter::ParameterVariant> parameters;
    // parameters.push_back(sim_time);

    // this->set_parameters(parameters);

    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);
    floor_robot_.setPlanningTime(10.0);
    floor_robot_.setNumPlanningAttempts(5);
    floor_robot_.allowReplanning(true);


    // Create callback groups
    auto move_robot_service_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto gripper_service_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
   

    // Create a service to pause the motion.
    // pause_robot_service_ = create_service<std_srvs::srv::Trigger>(
    //     "/robot/pause",
    //     std::bind(&FloorRobotNode::pauseRobotCallback, this, std::placeholders::_1, std::placeholders::_2));

    move_robot_service_ = create_service<PickPlaceSrv>(
        "/robot/move",
        std::bind(&FloorRobotNode::moveRobotCallback, this, std::placeholders::_1, std::placeholders::_2));

    //Create Clients
    enable_gripper_client_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
    change_gripper_tool_client_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
    // client to /ariac/perform_quality_check
    quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>(
      "/ariac/perform_quality_check");

    enable_gripper_client_ = this->create_client<VacuumGripperControlSrv>("/ariac/floor_robot_enable_gripper");
    change_gripper_tool_client_ = this->create_client<ChangeGripperSrv>("/ariac/floor_robot_change_gripper");

    // Create a SubscriptionOptions object
    auto subscription_options = rclcpp::SubscriptionOptions();
    rclcpp::SubscriptionOptions gripper_options;
    gripper_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    gripper_options.callback_group = gripper_cbg_; 

    subscription_options.callback_group = subscription_callback_group;
    //Create Gripper state subscriber
     gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
      "/ariac/floor_robot_gripper_state",
      rclcpp::QoS(rclcpp::KeepLast(1))
          .best_effort()
          .durability_volatile(),
      std::bind(&FloorRobotNode::floor_gripper_state_cb, this,
                std::placeholders::_1),
      gripper_options);
    
       // add all static models in ariac to the planning scene
       //For collision avoidance planning
    add_models_to_planning_scene();
    //Just logg that we have created the constructor
    RCLCPP_INFO(this->get_logger(), "Initialization of planning scene successful.");
////------////

    // Services timers to check after fix duration to call any service or not
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&FloorRobotNode::serviceTimerCallback, this),gripper_cbg_);


}
// // Destructor for the FloorRobot class
//  FloorRobotNode::~FloorRobotNode() {
//     // Call the destructor of the MoveGroupInterface object
//     // that is a member of the FloorRobot class
//     floor_robot_.~MoveGroupInterface();
// }

void FloorRobotNode::moveRobotCallback(
    const PickPlaceSrv::Request::SharedPtr request ,
    PickPlaceSrv::Response::SharedPtr response)
{
    //Sensor pose
      geometry_msgs::msg::Pose target_pose;
    // Define the target pose

    std::vector<geometry_msgs::msg::Pose> waypoints;

    float offset = 0.0;
    if (request->part_type!="")
        offset = utils::OFFSETS["part"] + utils::PART_HEIGHTS[request->part_type];
    else
    {
        offset = utils::OFFSETS["tray"]; //+ utils::PART_HEIGHTS[request->tray_id];
    } 

    // Check if the Gripper approriate for the task, if not change the gripper
    
    if (request->tray_id==-1 && ChangeGripperSrv::Request::PART_GRIPPER!=utils::GRIPPER_TYPE[floor_gripper_state_.type]){
        //move To Gripper Station
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to Gripper tool position");
        moveToGripperStation(request->gripper_station_pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "On the Station");
        // its part task, need part gripper
        // changeGripperTool(ChangeGripperSrv::Request::PART_GRIPPER);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Change Tool service called");
        _change_gripper_tool_service_started = true;
        _change_gripper_tool_service_value = ChangeGripperSrv::Request::PART_GRIPPER;
        while(!_change_gripper_tool_service_started){}
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Change Tool service succesfull");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to Gripper original pose");
        moveToGripperStation(request->gripper_station_pose,2,utils::OFFSETS["tool_loc"]);

    }
    if (request->part_type=="" && ChangeGripperSrv::Request::TRAY_GRIPPER!=utils::GRIPPER_TYPE[floor_gripper_state_.type])
    {
        //move To Gripper Station
        moveToGripperStation(request->gripper_station_pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "On the Station");
        // its part task, need part gripper
        // changeGripperTool(ChangeGripperSrv::Request::PART_GRIPPER);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Change Tool service called");
        // its part task, need part gripper
        // changeGripperTool(ChangeGripperSrv::Request::TRAY_GRIPPER);
        _change_gripper_tool_service_started = true;
        _change_gripper_tool_service_value = ChangeGripperSrv::Request::TRAY_GRIPPER;
        while(!_change_gripper_tool_service_started){}

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Change Tool service succesfull");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to Gripper original pose");
        moveToGripperStation(request->gripper_station_pose,2,utils::OFFSETS["tool_loc"]);
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

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot move successful");
            
            // auto request_vacuum_gripper = std::make_shared<VacuumGripperControlSrv::Request>();
            if (request->pick_place==PickPlaceSrv::Request::PICK){

                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Requested for Change Gripper state, enabling.");
                  _enable_gripper_service_started = true;
                  _enable_gripper_service_value = true;
                  while (!_enable_gripper_service_started){}
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Change Gripper state to enabled");
                 
                  geometry_msgs::msg::Pose object_pose = request->destination_pose;
                  std::string object_file = ""; 
                  std::string object_name = "";
                  if (request->tray_id==-1){
                    object_name = request->part_color + std::string("_") + request->part_type;
                    object_file = request->part_type + ".stl";
                    }
                  else{
                    object_name = "kit_tray_" + request->tray_id;
                    object_file = "kit_tray.stl";
                    }
                    RCLCPP_INFO(get_logger(), "object_name : %s, object_file : %s",object_name.c_str(),object_file.c_str());
                    add_single_model_to_planning_scene( object_name, object_file, object_pose);
                    floor_robot_.attachObject(object_name);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attached Object, %s",object_name.c_str());
                  }
             else {
                // changeGripperState(false);
                _enable_gripper_service_started = true;
                _enable_gripper_service_value = false;
                while (!_enable_gripper_service_started){}
                std::string object_file = ""; 
                std::string object_name = "";
                if (request->tray_id==-1){
                    object_name = request->part_color + std::string("_") + request->part_type;
                    object_file = request->part_type + ".stl";
                }
                else{
                    object_name = "kit_tray_" + request->tray_id;
                    object_file = "kit_tray.stl";
                }
                RCLCPP_INFO(get_logger(), "object_name : %s, object_file : %s",object_name.c_str(),object_file.c_str());
                // Drop object
                floor_robot_.detachObject(object_name);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detached Object, %s",object_name.c_str());

             }

            
//             _change_gripper_tool_service_started = true;
//             auto request_change_gripper = std::make_shared<ChangeGripperSrv::Request>();
//             if (request->tray_id==-1) request_change_gripper->gripper_type = ChangeGripperSrv::Request::PART_GRIPPER; //ariac_msg s::srv::ChangeGripper::Request::PART_GRIPPER;
//             else request_change_gripper->gripper_type = ChangeGripperSrv::Request::TRAY_GRIPPER;
//             change_gripper_tool_client_->async_send_request(request_change_gripper,std::bind(&FloorRobotNode::changeGripperTool, this, std::placeholders::_1));            
//             while (_change_gripper_tool_service_started ){continue;  
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


void FloorRobotNode::moveToGripperStation(geometry_msgs::msg::Pose target_pose, int depth, float offset){

    if (depth>=3) return;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"time %d to move the gripper",depth);

    target_pose.position.z += offset;
    // Convert roll, pitch, yaw to quaternion
    double roll = 0;   // Roll angle in radians
    double pitch = 3.14;   // Pitch angle in radians
    double yaw = 0;    // Yaw angle in radians

    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);

    // Set the orientation
    target_pose.orientation.x = orientation.x();
    target_pose.orientation.y = orientation.y();
    target_pose.orientation.z = orientation.z();
    target_pose.orientation.w = orientation.w();

    // // Set the target pose for the robot
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Robot moved successfully");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Trajectory execution failed with error code: %s",std::to_string(result.val).c_str());
        }
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Unable to generate trajectory");
    }
    moveToGripperStation(target_pose,depth+1,-offset);

//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     //  q.x = target_pose.orientation.x;
//     // waypoints.push_back(target_pose);
//     for(float i=0;i<offset;i+=0.01){waypoints.push_back(Utils::build_pose(
//       target_pose.position.x, target_pose.position.y, target_pose.position.z-i,target_pose.orientation));
//       }
  

  
//   if (!move_through_waypoints(waypoints, 0.2, 0.1))RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error Following trajectory");

}


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




bool FloorRobotNode::changeGripperState(bool enable){
  if (floor_gripper_state_.enabled == enable)
  {
    if (floor_gripper_state_.enabled)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Already enabled");
    else
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Already disabled");

    return false;
  }

  // Call enable service
  auto request = std::make_shared<VacuumGripperControlSrv::Request>();
  request->enable = enable;

  auto result = enable_gripper_client_->async_send_request(request);
  result.wait();

  if (!result.get()->success)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error calling gripper enable service");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeState - func - Vacuum Enabled");
  return true;
  _enable_gripper_service_started = false;
}
/**
 * This function is used to change the gripper type
*/
bool FloorRobotNode::changeGripperTool(uint8_t gripper_type){
    auto request = std::make_shared<ChangeGripperSrv::Request>();
    
    request->gripper_type = gripper_type;
    auto future = change_gripper_tool_client_->async_send_request(request);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service to get response");
    future.wait();
    auto response = future.get();

    if (response->success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully Change Gripper");
        return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error : %s",response->message.c_str());
    _change_gripper_tool_service_started = false;
    return false;
    _change_gripper_tool_service_started = false;
}

//=============================================//
/**
 * @brief add_single_model_to_planning_scene function
 * adds only objects of interest to the scene that changes for each order
 * This is will be used to dynamically add objects in the planning scene
 
*/
void FloorRobotNode::add_single_model_to_planning_scene(
    std::string name, std::string mesh_file,
    geometry_msgs::msg::Pose model_pose)
{
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("rwa5_2");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_.addCollisionObjects(collision_objects);
}

//=============================================//
void FloorRobotNode::add_models_to_planning_scene()
{
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {{"bin1", std::pair<double, double>(-1.9, 3.375)},
                                                                    {"bin2", std::pair<double, double>(-1.9, 2.625)},
                                                                    {"bin3", std::pair<double, double>(-2.65, 2.625)},
                                                                    {"bin4", std::pair<double, double>(-2.65, 3.375)},
                                                                    {"bin5", std::pair<double, double>(-1.9, -3.375)},
                                                                    {"bin6", std::pair<double, double>(-1.9, -2.625)},
                                                                    {"bin7", std::pair<double, double>(-2.65, -2.625)},
                                                                    {"bin8", std::pair<double, double>(-2.65, -3.375)}};

  geometry_msgs::msg::Pose bin_pose;
  for (auto const &bin : bin_positions)
  {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 3.14159);

    add_single_model_to_planning_scene(bin.first, "bin.stl", bin_pose);
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions = {
      {"as1", std::pair<double, double>(-7.3, 3)},
      {"as2", std::pair<double, double>(-12.3, 3)},
      {"as3", std::pair<double, double>(-7.3, -3)},
      {"as4", std::pair<double, double>(-12.3, -3)},
  };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const &station : assembly_station_positions)
  {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

    add_single_model_to_planning_scene(
        station.first, "assembly_station.stl", assembly_station_pose);
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
      {"as1_insert", std::pair<double, double>(-7.7, 3)},
      {"as2_insert", std::pair<double, double>(-12.7, 3)},
      {"as3_insert", std::pair<double, double>(-7.7, -3)},
      {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;
  for (auto const &insert : assembly_insert_positions)
  {
    assembly_insert_pose.position.x = insert.second.first;
    assembly_insert_pose.position.y = insert.second.second;
    assembly_insert_pose.position.z = 1.011;
    assembly_insert_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

    add_single_model_to_planning_scene(insert.first, "assembly_insert.stl",
                                       assembly_insert_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose = geometry_msgs::msg::Pose();
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene("conveyor", "conveyor.stl",
                                     conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 3.14159);

  add_single_model_to_planning_scene("kts1_table", "kit_tray_table.stl",
                                     kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene("kts2_table", "kit_tray_table.stl",
                                     kts2_table_pose);
}

//=============================================//
bool FloorRobotNode::move_to_target()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success)
  {
    return static_cast<bool>(floor_robot_.execute(plan));
  }
  else
  {
    // RCLCPP_ERROR(this->get_logger(), "Unable to generate plan");
    return false;
  }
}
///
// void FloorRobotNode::wait_for_attach_completion(double timeout)
// {
//   // Wait for part to be attached
//   rclcpp::Time start = now();
//   std::vector<geometry_msgs::msg::Pose> waypoints;
//   geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

//   while (!floor_gripper_state_.attached)
//   {
//     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
//                          "Waiting for gripper attach");

//     waypoints.clear();
//     starting_pose.position.z -= 0.001;
//     waypoints.push_back(starting_pose);

//     move_through_waypoints(waypoints, 0.1, 0.1);

//     usleep(200);

//     // if (floor_gripper_state_.attached)
//     //     return;

//     if (now() - start > rclcpp::Duration::from_seconds(timeout))
//     {
//       RCLCPP_ERROR(get_logger(), "Unable to pick up object");
//       return;
//     }
//   }
// }

bool FloorRobotNode::move_through_waypoints(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(
      floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_.execute(trajectory));
}

void FloorRobotNode::serviceTimerCallback(){
    try{
        if (_enable_gripper_service_started){
            changeGripperState(_enable_gripper_service_value);
            
        }
        if (_change_gripper_tool_service_started){
            auto val = changeGripperTool(_change_gripper_tool_service_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully tool changed :  %d",val);
            
        }
    }
    catch(std::exception &ex){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception time callback: %s",ex.what());
    }
}
///

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

