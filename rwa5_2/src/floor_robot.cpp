TestCompetitor::TestCompetitor()
    : Node("test_competitor"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
      ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
      planning_scene_()
{
  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);

  ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
  ceiling_robot_.setMaxVelocityScalingFactor(1.0);

  // Subscribe to topics
  rclcpp::SubscriptionOptions options;

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  options.callback_group = topic_cb_group_;

  
  // Initialize service clients
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
  AddModelsToPlanningScene();

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}
TestCompetitor::~TestCompetitor()
{
  floor_robot_.~MoveGroupInterface();
}
geometry_msgs::msg::Pose TestCompetitor::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
  KDL::Frame f1;
  KDL::Frame f2;

  tf2::fromMsg(p1, f1);
  tf2::fromMsg(p2, f2);

  KDL::Frame f3 = f1 * f2;

  return tf2::toMsg(f3);
}
void TestCompetitor::LogPose(geometry_msgs::msg::Pose p)
{
  tf2::Quaternion q(
      p.orientation.x,
      p.orientation.y,
      p.orientation.z,
      p.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  roll *= 180 / M_PI;
  pitch *= 180 / M_PI;
  yaw *= 180 / M_PI;

  RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
              p.position.x, p.position.y, p.position.z,
              roll, pitch, yaw);
}

geometry_msgs::msg::Pose TestCompetitor::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;

  return pose;
}

geometry_msgs::msg::Pose TestCompetitor::FrameWorldPose(std::string frame_id)
{
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try
  {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

double TestCompetitor::GetYaw(geometry_msgs::msg::Pose pose)
{
  tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

geometry_msgs::msg::Quaternion TestCompetitor::QuaternionFromRPY(double r, double p, double y)
{
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;

  q.setRPY(r, p, y);

  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  return q_msg;
}

void TestCompetitor::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
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