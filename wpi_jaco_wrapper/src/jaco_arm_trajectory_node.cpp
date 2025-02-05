#include <wpi_jaco_wrapper/jaco_arm_trajectory_node.h>

using namespace std;

using namespace jaco;

JacoArmTrajectoryController::JacoArmTrajectoryController(const std::shared_ptr<rclcpp::Node> nh):
  nh(nh)
{
  arm_initialized = false;

  kinova_gripper_ = true;
  if (!nh->has_parameter("kinova_gripper"))
    nh->declare_parameter("kinova_gripper", kinova_gripper_);
  nh->get_parameter("kinova_gripper", kinova_gripper_);

  loadParameters(nh);

  jc = std::make_shared<JacoConversions>(nh);
  jk = std::make_shared<JacoKinematics>(nh);

  boost::recursive_mutex::scoped_lock lock(api_mutex);

  RCLCPP_INFO(nh->get_logger(), "Trying to initialize JACO API...");
  while ( InitAPI() != NO_KINOVA_ERROR )
  {
    RCLCPP_ERROR(nh->get_logger(), "Could not initialize Kinova API. Is the arm connected?");
    RCLCPP_INFO(nh->get_logger(), "Retrying in 5 seconds..");
    rclcpp::Rate(5).sleep();
  }
  rclcpp::Rate(1).sleep();
  StartControlAPI();
  rclcpp::Rate(3).sleep();
  StopControlAPI();

  // Initialize arm
  bool home_arm = true;
  if (!nh->has_parameter("home_arm_on_init"))
      nh->declare_parameter("home_arm_on_init", home_arm);
  nh->get_parameter("home_arm_on_init", home_arm);

  if (home_arm)
    MoveHome();
  InitFingers();
  SetFrameType(0); //set end effector to move with respect to the fixed frame

  RCLCPP_INFO(nh->get_logger(), "Arm initialized.");

  // Initialize joint names
  if (arm_name_ == "jaco2")
  {
    joint_names.push_back("jaco_shoulder_pan_joint");
    joint_names.push_back("jaco_shoulder_lift_joint");
    joint_names.push_back("jaco_elbow_joint");
    joint_names.push_back("jaco_wrist_1_joint");
    joint_names.push_back("jaco_wrist_2_joint");
    joint_names.push_back("jaco_wrist_3_joint");
  }
  else
  {
    for (int joint_id = 0; joint_id < NUM_JACO_JOINTS; ++joint_id)
    {
      stringstream joint_name_stream;
      joint_name_stream << arm_name_ << "_joint_" << (joint_id + 1);
      string joint_name = joint_name_stream.str();
      joint_names.push_back(joint_name);
    }
  }
  if (kinova_gripper_)
  {
    finger_conv_ratio_ = 1.4; // for jaco v2 and all finger_conv_ratio_ = 80.0 / 6800.0;
    for (int finger_id = 0; finger_id < num_fingers_; ++finger_id)
    {
      stringstream finger_name_stream;
      finger_name_stream << arm_name_ << "_joint_finger_" << (finger_id + 1);
      string finger_name = finger_name_stream.str();
      joint_names.push_back(finger_name);
    }
    for (int finger_id = 0; finger_id < num_fingers_; ++finger_id)
    {
      stringstream finger_tip_name_stream;
      finger_tip_name_stream << arm_name_ << "_joint_finger_tip_" << (finger_id + 1);
      string finger_tip_name = finger_tip_name_stream.str();
      joint_names.push_back(finger_tip_name);
    }
  }

  StartControlAPI();
  SetAngularControl();
  controlType = ANGULAR_CONTROL;
  eStopEnabled = false;

  // Messages
  joint_state_pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "_arm/joint_states", 1);
  cartesianCmdPublisher = nh->create_publisher<wpi_jaco_msgs::msg::CartesianCommand>(topic_prefix_+"_arm/cartesian_cmd", 1);
  angularCmdPublisher = nh->create_publisher<wpi_jaco_msgs::msg::AngularCommand>(topic_prefix_+"_arm/angular_cmd", 1);
  // update_joint_states();
  armHomedPublisher = nh->create_publisher<std_msgs::msg::Bool>(topic_prefix_ + "_arm/arm_homed", 1);

  cartesianCmdSubscriber = nh->create_subscription<wpi_jaco_msgs::msg::CartesianCommand>(topic_prefix_+"_arm/cartesian_cmd", 1,
                          std::bind(&JacoArmTrajectoryController::cartesianCmdCallback, this, std::placeholders::_1));
  angularCmdSubscriber = nh->create_subscription<wpi_jaco_msgs::msg::AngularCommand>(topic_prefix_+"_arm/angular_cmd", 1,
                          std::bind(&JacoArmTrajectoryController::angularCmdCallback, this, std::placeholders::_1));

  // Services
  angularPositionServer = nh->create_service<wpi_jaco_msgs::srv::GetAngularPosition>(topic_prefix_+"_arm/get_angular_position", std::bind(&JacoArmTrajectoryController::getAngularPosition, this,
                                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  cartesianPositionServer = nh->create_service<wpi_jaco_msgs::srv::GetCartesianPosition>(topic_prefix_+"_arm/get_cartesian_position", std::bind(&JacoArmTrajectoryController::getCartesianPosition, this,
                                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  eStopServer = nh->create_service<wpi_jaco_msgs::srv::EStop>(topic_prefix_+"_arm/software_estop", std::bind(&JacoArmTrajectoryController::eStopCallback, this,
                                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  eraseTrajectoriesServer = nh->create_service<std_srvs::srv::Empty>(topic_prefix_+"_arm/erase_trajectories", std::bind(&JacoArmTrajectoryController::eraseTrajectoriesCallback, this,
                                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create actionlib servers and clients
  // trajectory_server_              = new TrajectoryServer( nh, topic_prefix_ + "_arm/arm_controller/trajectory", boost::bind(&JacoArmTrajectoryController::execute_trajectory, this, _1), true);

  // smooth_joint_trajectory_server_ = new TrajectoryServer( nh, topic_prefix_ + "_arm/joint_velocity_controller/trajectory", boost::bind(&JacoArmTrajectoryController::execute_joint_trajectory, this, _1), true);
  smooth_joint_trajectory_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      nh,
      topic_prefix_ + "_arm/joint_velocity_controller/trajectory",
      std::bind(&JacoArmTrajectoryController::joint_trajectory_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JacoArmTrajectoryController::joint_trajectory_handle_cancel, this, std::placeholders::_1),
      std::bind(&JacoArmTrajectoryController::joint_trajectory_handle_accepted, this, std::placeholders::_1));

  // smooth_trajectory_server_       = new TrajectoryServer( nh, topic_prefix_ + "_arm/smooth_arm_controller/trajectory", boost::bind(&JacoArmTrajectoryController::execute_smooth_trajectory, this, _1), false);
  home_arm_server_ = rclcpp_action::create_server<HomeArm>(
      nh,
      topic_prefix_ + "_arm/home_arm",
      std::bind(&JacoArmTrajectoryController::home_arm_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JacoArmTrajectoryController::home_arm_handle_cancel, this, std::placeholders::_1),
      std::bind(&JacoArmTrajectoryController::home_arm_handle_accepted, this, std::placeholders::_1));

  if (kinova_gripper_)
  {
    gripper_server_ = rclcpp_action::create_server<GripperCommand>(
      nh,
      topic_prefix_ + "_arm/fingers_controller/gripper",
      std::bind(&JacoArmTrajectoryController::execute_gripper_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JacoArmTrajectoryController::execute_gripper_handle_cancel, this, std::placeholders::_1),
      std::bind(&JacoArmTrajectoryController::execute_gripper_handle_accepted, this, std::placeholders::_1));
    
  //   gripper_server_radian_          = new GripperServer( nh, topic_prefix_ + "_arm/fingers_controller_radian/gripper", boost::bind(&JacoArmTrajectoryController::execute_gripper_radian, this, _1), false);
  }

  joint_state_timer_ = nh->create_wall_timer(std::chrono::milliseconds(33),
                                      std::bind(&JacoArmTrajectoryController::update_joint_states, this));

  if (home_arm)
  {
    //publish to arm_homed because the arm is initialized
    std_msgs::msg::Bool msg;
    msg.data = true;
    armHomedPublisher->publish(msg);
  }

  arm_initialized = true;
}

JacoArmTrajectoryController::~JacoArmTrajectoryController()
{
  StopControlAPI();
  CloseAPI();
}

/** Adjust angle to equivalent angle on [-pi, pi]
 *  @param angle the angle to be simplified (-inf, inf)
 *  @return the simplified angle on [-pi, pi]
 */
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

void JacoArmTrajectoryController::update_joint_states()
{
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    AngularPosition force_data;
    GetAngularForce(force_data);
    joint_eff_[0] = force_data.Actuators.Actuator1;
    joint_eff_[1] = force_data.Actuators.Actuator2;
    joint_eff_[2] = force_data.Actuators.Actuator3;
    joint_eff_[3] = force_data.Actuators.Actuator4;
    joint_eff_[4] = force_data.Actuators.Actuator5;
    joint_eff_[5] = force_data.Actuators.Actuator6;
    if (kinova_gripper_)
    {
      joint_eff_[6] = force_data.Fingers.Finger1;
      joint_eff_[7] = force_data.Fingers.Finger2;
      joint_eff_[8] = force_data.Fingers.Finger3;
      for (int i = 9; i<12; i++) {
        joint_eff_[i] = 0;
      }
    }

    AngularPosition velocity_data;
    GetAngularVelocity(velocity_data);
    joint_vel_[0] = velocity_data.Actuators.Actuator1 * DEG_TO_RAD;
    joint_vel_[1] = velocity_data.Actuators.Actuator2 * DEG_TO_RAD;
    joint_vel_[2] = velocity_data.Actuators.Actuator3 * DEG_TO_RAD;
    joint_vel_[3] = velocity_data.Actuators.Actuator4 * DEG_TO_RAD;
    joint_vel_[4] = velocity_data.Actuators.Actuator5 * DEG_TO_RAD;
    joint_vel_[5] = velocity_data.Actuators.Actuator6 * DEG_TO_RAD;
    if (kinova_gripper_)
    {
      // no velocity info for fingers
      for (int i = 6; i<12; i++) {
        joint_vel_[i] = 0;
      }
    }

    AngularPosition position_data;
    GetAngularPosition(position_data);
    joint_pos_[0] = simplify_angle(position_data.Actuators.Actuator1 * DEG_TO_RAD);
    joint_pos_[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
    joint_pos_[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
    joint_pos_[3] = simplify_angle(position_data.Actuators.Actuator4 * DEG_TO_RAD);
    joint_pos_[4] = simplify_angle(position_data.Actuators.Actuator5 * DEG_TO_RAD);
    joint_pos_[5] = simplify_angle(position_data.Actuators.Actuator6 * DEG_TO_RAD);
    if (kinova_gripper_)
    {
      // This logic is in kinova-ros package
      CartesianPosition cartesian_position;
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetCartesianPosition(cartesian_position);
      }
      joint_pos_[6] = cartesian_position.Fingers.Finger1 * DEG_TO_RAD * finger_scale_;
      joint_pos_[7] = cartesian_position.Fingers.Finger2 * DEG_TO_RAD * finger_scale_;
      joint_pos_[8] = cartesian_position.Fingers.Finger3 * DEG_TO_RAD * finger_scale_;
      for (int i = 9; i<12; i++) {
        joint_pos_[i] = 0;
      }
    }

  }

  sensor_msgs::msg::JointState state;
  state.header.stamp = nh->get_clock()->now();
  state.name = joint_names;
  state.position.assign(joint_pos_.begin(), joint_pos_.end());
  state.velocity.assign(joint_vel_.begin(), joint_vel_.end());
  state.effort.assign(joint_eff_.begin(), joint_eff_.end());
  joint_state_pub_->publish(state);
}

/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 */
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

/*****************************************/
/**********  Trajectory Control **********/
/*****************************************/

void JacoArmTrajectoryController::execute_trajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> gh)
{
  const auto goal = gh->get_goal();
  if ( not arm_initialized )
  {
    return; // The arm is not fully initialized yet
  }
  //cancel check
  if (eStopEnabled)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
    gh->succeed(result);
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();
  }

  // update_joint_states();
  double current_joint_pos[NUM_JACO_JOINTS];

  AngularPosition position_data;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetAngularPosition(position_data);
  }
  current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
  current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
  current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
  current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
  current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
  current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;

  //initialize trajectory point
  TrajectoryPoint trajPoint;
  trajPoint.InitStruct();
  trajPoint.Position.Type = ANGULAR_POSITION;
  trajPoint.Position.HandMode = HAND_NOMOVEMENT;
  BOOST_FOREACH(trajectory_msgs::msg::JointTrajectoryPoint point, goal->trajectory.points)
  {
    RCLCPP_INFO(nh->get_logger(), "Trajectory Point");
    double joint_cmd[NUM_JACO_JOINTS];
    for (int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); ++trajectory_index)
    {
      string joint_name = goal->trajectory.joint_names[trajectory_index];
      int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
      if (joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
      {
        //RCLCPP_INFO(nh->get_logger(), "Before: %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
        if (joint_index != 1 && joint_index != 2)
          joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]),
                                                      current_joint_pos[joint_index]);
        else
          joint_cmd[joint_index] = point.positions[trajectory_index];
        //RCLCPP_INFO(nh->get_logger(), "After:  %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, joint_cmd[joint_index]);
      }
    }
    for (int i = 0; i < NUM_JACO_JOINTS; i++)
      current_joint_pos[i] = joint_cmd[i];

    AngularInfo angles;
    angles.Actuator1 = joint_cmd[0] * RAD_TO_DEG;
    angles.Actuator2 = joint_cmd[1] * RAD_TO_DEG;
    angles.Actuator3 = joint_cmd[2] * RAD_TO_DEG;
    angles.Actuator4 = joint_cmd[3] * RAD_TO_DEG;
    angles.Actuator5 = joint_cmd[4] * RAD_TO_DEG;
    angles.Actuator6 = joint_cmd[5] * RAD_TO_DEG;

    trajPoint.Position.Actuators = angles;

    executeAngularTrajectoryPoint(trajPoint, false);
  }

  rclcpp::Rate rate(10);
  int trajectory_size;
  while (trajectory_size > 0)
  {
    //cancel check
    if (eStopEnabled)
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
      gh->succeed(result);
      return;
    }

    //check for preempt requests from clients
    if (gh->is_canceling() || !rclcpp::ok())
    {
      //stop gripper control
      trajPoint.Position.Type = ANGULAR_VELOCITY;
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

      //preempt action server
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      gh->canceled(result);
      RCLCPP_INFO(nh->get_logger(), "Joint trajectory server preempted by client");

      return;
    }

    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);

      TrajectoryFIFO Trajectory_Info;
      memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
      GetGlobalTrajectoryInfo(Trajectory_Info);
      trajectory_size = Trajectory_Info.TrajectoryCount;
    }
    //RCLCPP_INFO(nh->get_logger(), "%f, %f, %f, %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
    rate.sleep();
  }

  //stop gripper control
  trajPoint.Position.Type = ANGULAR_VELOCITY;
  trajPoint.Position.Actuators.Actuator1 = 0.0;
  trajPoint.Position.Actuators.Actuator2 = 0.0;
  trajPoint.Position.Actuators.Actuator3 = 0.0;
  trajPoint.Position.Actuators.Actuator4 = 0.0;
  trajPoint.Position.Actuators.Actuator5 = 0.0;
  trajPoint.Position.Actuators.Actuator6 = 0.0;
  executeAngularTrajectoryPoint(trajPoint, true);

  RCLCPP_INFO(nh->get_logger(), "Trajectory Control Complete.");
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
  gh->succeed(result);
}

/*****************************************/
/*****  Smoothed Trajectory Control ******/
/*****************************************/

void JacoArmTrajectoryController::joint_trajectory_handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory>gh)
{
  std::thread
  {
    // std::bind(&JacoArmTrajectoryController::execute_smooth_trajectory, this, std::placeholders::_1), gh
    std::bind(&JacoArmTrajectoryController::execute_joint_trajectory, this, std::placeholders::_1), gh
    // std::bind(&JacoArmTrajectoryController::execute_trajectory, this, std::placeholders::_1), gh
  }.detach();
}

rclcpp_action::GoalResponse JacoArmTrajectoryController::joint_trajectory_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal>goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JacoArmTrajectoryController::joint_trajectory_handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory>gh)
{
  RCLCPP_INFO(nh->get_logger(), "Received request to cancel goal");
  (void)gh;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JacoArmTrajectoryController::execute_smooth_trajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> gh)
{
  const auto goal = gh->get_goal();
  //cancel check
  if (eStopEnabled)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
    gh->succeed(result);
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();
  }

  // update_joint_states();
  double current_joint_pos[NUM_JACO_JOINTS];

  AngularPosition position_data;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetAngularPosition(position_data);
  }
  current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
  current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
  current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
  current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
  current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
  current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;

  TrajectoryPoint trajPoint;
  trajPoint.InitStruct();
  trajPoint.Position.Type = CARTESIAN_POSITION;
  trajPoint.Position.HandMode = HAND_NOMOVEMENT;

  BOOST_FOREACH(trajectory_msgs::msg::JointTrajectoryPoint point, goal->trajectory.points)
  {
    double joint_cmd[NUM_JACO_JOINTS];
    for (int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); ++trajectory_index)
    {
      string joint_name = goal->trajectory.joint_names[trajectory_index];
      int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
      if (joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
      {
        //convert angles from continuous joints to be correct for the arm API
        if (joint_index != 1 && joint_index != 2)
          joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]),
                                                      current_joint_pos[joint_index]);
        else
          joint_cmd[joint_index] = point.positions[trajectory_index];
      }
    }

    for (int i = 0; i < NUM_JACO_JOINTS; i++)
      current_joint_pos[i] = joint_cmd[i];

    //convert joint angles to end effector pose
    wpi_jaco_msgs::msg::Joints arm_joints;
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      arm_joints.joints.push_back(joint_cmd[i]);
    }
    try
    {
      auto fk = jk->callFK(arm_joints);
      //conversion to rpy

      auto euler_angles = jc->QuaternionToEuler(fk.pose.orientation);

      trajPoint.Position.CartesianPosition.X = fk.pose.position.x;
      trajPoint.Position.CartesianPosition.Y = fk.pose.position.y;
      trajPoint.Position.CartesianPosition.Z = fk.pose.position.z;
      trajPoint.Position.CartesianPosition.ThetaX = euler_angles.roll;
      trajPoint.Position.CartesianPosition.ThetaY = euler_angles.pitch;
      trajPoint.Position.CartesianPosition.ThetaZ = euler_angles.yaw;

      // for debugging:
      RCLCPP_INFO(nh->get_logger(), "Trajectory point: (%f, %f, %f); (%f, %f, %f)", trajPoint.Position.CartesianPosition.X, trajPoint.Position.CartesianPosition.Y, trajPoint.Position.CartesianPosition.Z, trajPoint.Position.CartesianPosition.ThetaX, trajPoint.Position.CartesianPosition.ThetaY, trajPoint.Position.CartesianPosition.ThetaZ);

      //send point to arm trajectory
      executeCartesianTrajectoryPoint(trajPoint, false);
    }
    catch (std::exception& e)
    {
      RCLCPP_INFO(nh->get_logger(), "Quaternion to Euler Angle conversion failed: ", e.what());

      trajPoint.Position.Type = ANGULAR_VELOCITY;
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_JOINTS;
      gh->succeed(result);
      return;
    }
  }

  //wait for trajectory to complete execution
  rclcpp::Rate rate(10);
  int trajectory_size;
  int initialTrajectorySize;
  TrajectoryFIFO Trajectory_Info;
  memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetGlobalTrajectoryInfo(Trajectory_Info);
  }
  trajectory_size = Trajectory_Info.TrajectoryCount;
  initialTrajectorySize = trajectory_size;
  while (trajectory_size > 0)
  {
    //cancel check
    if (eStopEnabled)
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
      gh->succeed(result);
      return;
    }

    //check for preempt requests from clients
    if (gh->is_canceling() || !rclcpp::ok())
    {
      //stop gripper control
      trajPoint.Position.Type = ANGULAR_VELOCITY;
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

      //preempt action server
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      gh->canceled(result);
      RCLCPP_INFO(nh->get_logger(), "Smooth trajectory server preempted by client");

      return;
    }

    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      GetGlobalTrajectoryInfo(Trajectory_Info);
    }
    trajectory_size = Trajectory_Info.TrajectoryCount;

    RCLCPP_INFO(nh->get_logger(), "%f, %f, %f, %f, %f, %f", joint_pos_[0], joint_pos_[1], joint_pos_[2], joint_pos_[3], joint_pos_[4], joint_pos_[5]);
    RCLCPP_INFO(nh->get_logger(), "Trajectory points complete: %d; remaining: %d", initialTrajectorySize - trajectory_size, trajectory_size);
    rate.sleep();
  }
  RCLCPP_INFO(nh->get_logger(), "Trajectory Control Complete.");
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
  gh->succeed(result);
}

void JacoArmTrajectoryController::execute_joint_trajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> gh)
{
  const auto goal = gh->get_goal();
  //check for cancel
  if (eStopEnabled)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
    gh->succeed(result);
    return;
  }

  float trajectoryPoints[NUM_JACO_JOINTS][goal->trajectory.points.size()];
  int numPoints = goal->trajectory.points.size();

  if (numPoints < 2)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_GOAL;
    gh->succeed(result);
    return;
  }

  //get trajectory data
  for (unsigned int i = 0; i < numPoints; i++)
  {

    //TEST
    for (int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); trajectory_index++)
    {
      string joint_name = goal->trajectory.joint_names[trajectory_index];
      int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
      if (joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
      {
        trajectoryPoints[joint_index][i] = goal->trajectory.points.at(i).positions.at(trajectory_index);
      }
    }
    //END TEST

    /*
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      trajectoryPoints[j][i] = goal->trajectory.points.at(i).positions.at(j);
    }
    */
  }

  //initialize arrays needed to fit a smooth trajectory to the given points
  ecl::Array<double> timePoints(numPoints);
  timePoints[0] = 0.0;
  vector<ecl::Array<double> > jointPoints;
  jointPoints.resize(NUM_JACO_JOINTS);
  float prevPoint[NUM_JACO_JOINTS];
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    jointPoints[i].resize(numPoints);
    jointPoints[i][0] = trajectoryPoints[i][0];
    prevPoint[i] = trajectoryPoints[i][0];
  }

  RCLCPP_INFO(nh->get_logger(), "Number of trajectory points: %d", numPoints);
  //determine time component of trajectories for each joint
  for (unsigned int i = 1; i < numPoints; i++)
  {
    float maxTime = 0.0;
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      //calculate approximate time required to move to the next position
      float time = fabs(trajectoryPoints[j][i] - prevPoint[j]);
      if (j <= 2)
        time /= LARGE_ACTUATOR_VELOCITY;
      else
        time /= SMALL_ACTUATOR_VELOCITY;

      if (time > maxTime)
        maxTime = time;

      jointPoints[j][i] = trajectoryPoints[j][i];
      prevPoint[j] = trajectoryPoints[j][i];
    }

    timePoints[i] = timePoints[i - 1] + maxTime * TIME_SCALING_FACTOR;
  }

  vector<ecl::SmoothLinearSpline> splines;
  splines.resize(6);
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], max_curvature_);
    splines.at(i) = tempSpline;
  }

  //control loop
  bool trajectoryComplete = false;
  double startTime = nh->get_clock()->now().seconds();
  double t = 0;
  float error[NUM_JACO_JOINTS];
  float totalError;
  float prevError[NUM_JACO_JOINTS] = {0};
  float currentPoint;
  double current_joint_pos[NUM_JACO_JOINTS];
  AngularPosition position_data;
  TrajectoryPoint trajPoint;
  trajPoint.InitStruct();
  trajPoint.Position.Type = ANGULAR_VELOCITY;
  trajPoint.Position.HandMode = HAND_NOMOVEMENT;

  rclcpp::Rate rate(600);

  while (!trajectoryComplete)
  {
    if (eStopEnabled)
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
      gh->succeed(result);
      return;
    }

    //check for preempt requests from clients
    if (gh->is_canceling())
    {
      //stop gripper control
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

      //preempt action server
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      gh->canceled(result);
      RCLCPP_INFO(nh->get_logger(), "Joint trajectory server preempted by client");

      return;
    }

    //get time for trajectory
    t = nh->get_clock()->now().seconds() - startTime;
    if (t > timePoints.at(timePoints.size() - 1))
    {
      //use final trajectory point as the goal to calculate error until the error
      //is small enough to be considered successful
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetAngularPosition(position_data);
      }
      current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;

      totalError = 0;
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        error[i] = nearest_equivalent(simplify_angle((splines.at(i))(timePoints.at(timePoints.size() - 1))),
                                      currentPoint) - currentPoint;
        totalError += fabs(error[i]);
      }

      if (totalError < .03)
      {
        trajPoint.Position.Actuators.Actuator1 = 0.0;
        trajPoint.Position.Actuators.Actuator2 = 0.0;
        trajPoint.Position.Actuators.Actuator3 = 0.0;
        trajPoint.Position.Actuators.Actuator4 = 0.0;
        trajPoint.Position.Actuators.Actuator5 = 0.0;
        trajPoint.Position.Actuators.Actuator6 = 0.0;
        executeAngularTrajectoryPoint(trajPoint, true);
        trajectoryComplete = true;
        RCLCPP_INFO(nh->get_logger(), "Trajectory complete!");
        break;
      }
    }
    else
    {
      //calculate error
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetAngularPosition(position_data);
      }
      current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        error[i] = nearest_equivalent(simplify_angle((splines.at(i))(t)), currentPoint) - currentPoint;
      }
    }

    //calculate control input
    //populate the velocity command
    trajPoint.Position.Actuators.Actuator1 = (KP * error[0] + KV * (error[0] - prevError[0]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator2 = (KP * error[1] + KV * (error[1] - prevError[1]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator3 = (KP * error[2] + KV * (error[2] - prevError[2]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator4 = (KP * error[3] + KV * (error[3] - prevError[3]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator5 = (KP * error[4] + KV * (error[4] - prevError[4]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator6 = (KP * error[5] + KV * (error[5] - prevError[5]) * RAD_TO_DEG);

    //for debugging:
    // cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;

    //send the velocity command
    executeAngularTrajectoryPoint(trajPoint, true);

    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      prevError[i] = error[i];
    }

    rate.sleep();
  }


  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
  gh->succeed(result);
}

/*****************************************/
/***********  Gripper Control ************/
/*****************************************/

void JacoArmTrajectoryController::execute_gripper_handle_accepted(const std::shared_ptr<GoalHandleGripperCommand>gh)
{
  std::thread
  {
    std::bind(&JacoArmTrajectoryController::execute_gripper, this, std::placeholders::_1), gh
  }.detach();
}

rclcpp_action::GoalResponse JacoArmTrajectoryController::execute_gripper_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GripperCommand::Goal>goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JacoArmTrajectoryController::execute_gripper_handle_cancel(const std::shared_ptr<GoalHandleGripperCommand>gh)
{
  RCLCPP_INFO(nh->get_logger(), "Received request to cancel goal");
  (void)gh;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JacoArmTrajectoryController::setFingerPositions(const std::vector<float> &fingers, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex);

    if (eStopEnabled)
    {
        RCLCPP_INFO(rclcpp::get_logger("kinova_comm"), "The fingers could not be set because the arm is stopped");
        return;
    }

    int result = NO_KINOVA_ERROR;
    int control_type;
    result=GetControlType(control_type); // are we currently in angular or Cartesian mode? Response	0 = Cartesian control type, 1 = Angular control type.

    //initialize the trajectory point. same initialization for an angular or Cartesian point
    TrajectoryPoint kinova_point;

    if (result != NO_KINOVA_ERROR)
    {
        throw std::runtime_error("Could not get the current control type");
    }
    else
    {
	if (push)
    	{
        	result = EraseAllTrajectories();
        	if (result != NO_KINOVA_ERROR)
        	{
           		throw std::runtime_error("Could not erase trajectories");
        	}
    	}
	// Initialize Cartesian control of the fingers
	kinova_point.Position.HandMode = POSITION_MODE;
	kinova_point.Position.Fingers.Finger1 = 50;//fingers[0];
	kinova_point.Position.Fingers.Finger2 = 50;//fingers[1];
	kinova_point.Position.Fingers.Finger3 = 50;//fingers[2];
	kinova_point.Position.Delay = 0.0;
	kinova_point.LimitationsActive = 0;
	if(control_type==0) //Cartesian
	{
		kinova_point.Position.Type = CARTESIAN_POSITION;
		CartesianPosition pose;
                memset(&pose, 0, sizeof(pose));  // zero structure
		result = GetCartesianCommand(pose);
    		if (result != NO_KINOVA_ERROR)
    		{
        		throw std::runtime_error("Could not get the Cartesian position");
    		}
		kinova_point.Position.CartesianPosition=pose.Coordinates;
	}
        else if(control_type==1) //angular
	{
		kinova_point.Position.Type = ANGULAR_POSITION;
		AngularPosition joint_angles;
    		memset(&joint_angles, 0, sizeof(joint_angles));  // zero structure
		result = GetAngularCommand(joint_angles);
    		if (result != NO_KINOVA_ERROR)
    		{
        		throw std::runtime_error("Could not get the angular position");
    		}
		kinova_point.Position.Actuators = joint_angles.Actuators;
	}
	else
	{
		throw std::runtime_error("Wrong control type");
	}
    }


    // getAngularPosition will cause arm drop
    // result = kinova_api_.getAngularPosition(joint_angles);

    result = SendBasicTrajectory(kinova_point);
    if (result != NO_KINOVA_ERROR)
    {
        throw std::runtime_error("Could not send advanced finger trajectory");
    }
}

void JacoArmTrajectoryController::execute_gripper(const std::shared_ptr<GoalHandleGripperCommand> gh)
{
  const auto goal = gh->get_goal();
  RCLCPP_INFO_STREAM(nh->get_logger(), "Gripper goal: " << goal->command.position * RAD_TO_DEG);

  //check for cancel
  if (eStopEnabled)
  {
    auto result = std::make_shared<GripperCommand::Result>();
    result->reached_goal = false;
    gh->succeed(result);
    return;
  }

  // auto feedback = std::make_shared<SetFingersPosition::Feedback>();
  // auto result = std::make_shared<SetFingersPosition::Result>();
  std::vector<float> current_finger_positions;
  rclcpp::Time current_time = nh->get_clock()->now();

  try
  {
      CartesianPosition cartesian_position;
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetCartesianPosition(cartesian_position);
      }
      current_finger_positions = {cartesian_position.Fingers.Finger1,cartesian_position.Fingers.Finger2,cartesian_position.Fingers.Finger3};

      last_nonstall_time_ = current_time;
      last_nonstall_finger_positions_ = current_finger_positions;

      std::vector<float> target;
      for (int i=0; i<3; i++)
        target.push_back(goal->command.position * RAD_TO_DEG);

      RCLCPP_INFO_STREAM(nh->get_logger(), "setting finger angles to: " << target[0] << "," << target[1] << "," << target[2]);
      setFingerPositions(target, 0, false);

      auto result = std::make_shared<GripperCommand::Result>();

      // Loop until the action completed, is preempted, or fails in some way.
      // timeout is left to the caller since the timeout may greatly depend on
      // the context of the movement.
      while (rclcpp::ok())
      {
          if (eStopEnabled)
          {
            result->reached_goal = false;
            gh->succeed(result);
            return;
          }
          else if (gh->is_canceling() || !rclcpp::ok())
          {
              // result->fingers = current_finger_positions.constructFingersMsg();
              gh->canceled(result);
              RCLCPP_DEBUG_STREAM(nh->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
              return;
          }

          CartesianPosition cartesian_position;
          {
            boost::recursive_mutex::scoped_lock lock(api_mutex);
            GetCartesianPosition(cartesian_position);
          }

          current_finger_positions = {cartesian_position.Fingers.Finger1, cartesian_position.Fingers.Finger2, cartesian_position.Fingers.Finger3};

          current_time = nh->get_clock()->now();
          // feedback->fingers = current_finger_positions.constructFingersMsg();

          double tolerance = 6400.0*0.01;
          // ToDo:
          // if (target.isCloseToOther(current_finger_positions, tolerance))
          // {
          //     // Check if the action has succeeeded
          //     // result->fingers = current_finger_positions.constructFingersMsg();
          //     gh->succeed(result);
          //     RCLCPP_DEBUG_STREAM(nh->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
          //     return;
          // }
          // else if (!last_nonstall_finger_positions_.isCloseToOther(current_finger_positions, stall_threshold_))
          // {
          //     // Check if we are outside of a potential stall condition
          //     last_nonstall_time_ = current_time;
          //     last_nonstall_finger_positions_ = current_finger_positions;
          // }
          // else if ((current_time.seconds() - last_nonstall_time_.seconds()) > stall_interval_seconds_)
          // {
          //     gh->abort(result);
          //     RCLCPP_DEBUG_STREAM(nh->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", Trajectory command failed ");
          //     return;
          // }

          rclcpp::Rate(10).sleep();
      }
  }
  catch(const std::exception& e)
  {
      // result->fingers = current_finger_positions.constructFingersMsg();
      RCLCPP_ERROR_STREAM(nh->get_logger(), e.what());
      auto result = std::make_shared<GripperCommand::Result>();
      gh->abort(result);
      RCLCPP_DEBUG_STREAM(nh->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
  }
}

void JacoArmTrajectoryController::execute_gripper_radian(const std::shared_ptr<GoalHandleGripperCommand> gh)
{
  const auto goal = gh->get_goal();
  //check for cancel
  if (eStopEnabled)
  {
    auto result = std::make_shared<GripperCommand::Result>();
    result->reached_goal = false;
    gh->succeed(result);
    return;
  }
  wpi_jaco_msgs::msg::AngularCommand cmd;
  cmd.position = true;
  cmd.arm_command = false;
  cmd.finger_command = true;
  cmd.repeat = false;
  cmd.fingers.resize(num_fingers_);
  for (int i = 0 ; i < num_fingers_ ; i++)
    cmd.fingers[i] = (gripper_closed_/.93)*goal->command.position;

  angularCmdPublisher->publish(cmd);

  //give the fingers sufficient time to start moving, this prevents early termination if a command is slow to reach arm
  rclcpp::Rate startupRate(1);
  startupRate.sleep();

  rclcpp::Rate rate(10);
  bool gripperMoving = true;
  while (gripperMoving)
  {
    //check for cancel
    if (eStopEnabled)
    {
      auto result = std::make_shared<GripperCommand::Result>();
      result->reached_goal = false;
      gh->succeed(result);
      return;
    }

    rate.sleep();
    //check for preempt requests from clients
    if (gh->is_canceling() || !rclcpp::ok())
    {
      //stop gripper control
      cmd.position = false;
      for (int i = 0 ; i < num_fingers_ ; i++)
        cmd.fingers[i] = 0.0;

      angularCmdPublisher->publish(cmd);

      //preempt action server
      RCLCPP_INFO(nh->get_logger(), "Gripper action server preempted by client");
      auto result = std::make_shared<GripperCommand::Result>();
      gh->canceled(result);

      return;
    }

    //see if fingers are still moving
    AngularPosition velocity_data;
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      GetAngularVelocity(velocity_data);
    }

    float totalSpeed = fabs(velocity_data.Fingers.Finger1) 
                     + fabs(velocity_data.Fingers.Finger2);
    if ( num_fingers_ == 3 )
      totalSpeed += fabs(velocity_data.Fingers.Finger3);

    if (totalSpeed <= 0.01)
      gripperMoving = false;

  }

  //stop gripper control
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();
  }

  cmd.position = false;
  for (int i = 0 ; i < num_fingers_ ; i++)
    cmd.fingers[i] = 0.0;

  angularCmdPublisher->publish(cmd);

  auto result = std::make_shared<GripperCommand::Result>();
  AngularPosition force_data;
  AngularPosition position_data;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetAngularPosition(position_data);
    GetAngularForce(force_data);
  }
  float finalError    = fabs(goal->command.position - position_data.Fingers.Finger1) 
                      + fabs(goal->command.position - position_data.Fingers.Finger2);
  if ( num_fingers_ == 3 )
    finalError += fabs(goal->command.position - position_data.Fingers.Finger3);

  RCLCPP_INFO(nh->get_logger(), "Final error: %f", finalError);
  result->reached_goal = (finalError <= finger_error_threshold_);
  result->position     = position_data.Fingers.Finger1 * DEG_TO_RAD * finger_scale_;
  result->effort       = force_data.Fingers.Finger1;
  result->stalled      = false;
  gh->succeed(result);
}

/*****************************************/
/**********  Other Arm Actions  **********/
/*****************************************/

void JacoArmTrajectoryController::home_arm_handle_accepted(const std::shared_ptr<GoalHandleHomeArm>gh)
{
  std::thread
  {
    std::bind(&JacoArmTrajectoryController::home_arm, this, std::placeholders::_1), gh
  }.detach();
}

rclcpp_action::GoalResponse JacoArmTrajectoryController::home_arm_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const HomeArm::Goal>goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JacoArmTrajectoryController::home_arm_handle_cancel(const std::shared_ptr<GoalHandleHomeArm>gh)
{
  RCLCPP_INFO(nh->get_logger(), "Received request to cancel goal");
  (void)gh;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JacoArmTrajectoryController::home_arm(const std::shared_ptr<GoalHandleHomeArm> gh)
{
  const auto goal = gh->get_goal();
  //check for cancel
  if (eStopEnabled)
  {
    auto result = std::make_shared<HomeArm::Result>();
    result->success = false;
    gh->succeed(result);
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    StopControlAPI();
    MoveHome();
    StartControlAPI();
    std_msgs::msg::Bool msg;
    msg.data = true;
    armHomedPublisher->publish(msg);
  }

  if (goal->retract)
  {
    //check for cancel
    if (eStopEnabled)
    {
      auto result = std::make_shared<HomeArm::Result>();
      result->success = false;
      gh->succeed(result);
      return;
    }

    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      //retract to given position
      controlType = ANGULAR_CONTROL;
      SetAngularControl();
    }

    angularCmdPublisher->publish(goal->retract_position);

    rclcpp::Rate rate(10);
    int trajectory_size = 1;
    while (trajectory_size > 0)
    {
      //check for cancel
      if (eStopEnabled)
      {
        auto result = std::make_shared<HomeArm::Result>();
        result->success = false;
        gh->succeed(result);
        return;
      }

      //check for preempt requests from clients
      if (gh->is_canceling() || !rclcpp::ok())
      {
        //preempt action server
        RCLCPP_INFO(nh->get_logger(), "Home arm server action server preempted by client");
        auto result = std::make_shared<HomeArm::Result>();
        gh->canceled(result);

        return;
      }

      TrajectoryFIFO Trajectory_Info;
      memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetGlobalTrajectoryInfo(Trajectory_Info);
      }
      trajectory_size = Trajectory_Info.TrajectoryCount;
      rate.sleep();
    }

    std_msgs::msg::Bool msg;
    msg.data = true;
    armHomedPublisher->publish(msg);
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    //set control type to previous value
    if (controlType == ANGULAR_CONTROL)
      SetAngularControl();
    else
      SetCartesianControl();
  }

  auto result = std::make_shared<HomeArm::Result>();
  result->success = true;
  gh->succeed(result);
}

bool JacoArmTrajectoryController::loadParameters(const std::shared_ptr<rclcpp::Node> n)
{
    RCLCPP_DEBUG(n->get_logger(), "Loading parameters");

    arm_name_ = std::string("jaco");
    if (!n->has_parameter("wpi_jaco/arm_name"))
        n->declare_parameter("wpi_jaco/arm_name", arm_name_);
    n->get_parameter("wpi_jaco/arm_name", arm_name_);

    finger_scale_ = 1.0;
    if (!n->has_parameter("wpi_jaco/finger_scale"))
        n->declare_parameter("wpi_jaco/finger_scale", finger_scale_);
    n->get_parameter("wpi_jaco/finger_scale", finger_scale_);

    finger_error_threshold_ = 1.0;
    if (!n->has_parameter("wpi_jaco/finger_error_threshold"))
        n->declare_parameter("wpi_jaco/finger_error_threshold", finger_error_threshold_);
    n->get_parameter("wpi_jaco/finger_error_threshold", finger_error_threshold_);

    gripper_open_ = 0.0;
    if (!n->has_parameter("wpi_jaco/gripper_open"))
        n->declare_parameter("wpi_jaco/gripper_open", gripper_open_);
    n->get_parameter("wpi_jaco/gripper_open", gripper_open_);

    gripper_closed_ = 65.0;
    if (!n->has_parameter("wpi_jaco/gripper_closed"))
        n->declare_parameter("wpi_jaco/gripper_closed", gripper_closed_);
    n->get_parameter("wpi_jaco/gripper_closed", gripper_closed_);

    max_curvature_ = 20.0;
    if (!n->has_parameter("wpi_jaco/max_curvature"))
        n->declare_parameter("wpi_jaco/max_curvature", max_curvature_);
    n->get_parameter("wpi_jaco/max_curvature", max_curvature_);

    max_speed_finger_ = 30.0;
    if (!n->has_parameter("wpi_jaco/max_speed_finger"))
        n->declare_parameter("wpi_jaco/max_speed_finger", max_speed_finger_);
    n->get_parameter("wpi_jaco/max_speed_finger", max_speed_finger_);

    num_fingers_ = 3;
    if (!n->has_parameter("wpi_jaco/num_fingers"))
        n->declare_parameter("wpi_jaco/num_fingers", num_fingers_);
    n->get_parameter("wpi_jaco/num_fingers", num_fingers_);

    RCLCPP_INFO(n->get_logger(), "arm_name: %s",                arm_name_.c_str());
    RCLCPP_INFO(n->get_logger(), "finger_scale: %f",            finger_scale_);
    RCLCPP_INFO(n->get_logger(), "finger_error_threshold_: %f", finger_error_threshold_);
    RCLCPP_INFO(n->get_logger(), "gripper_open_: %f",           gripper_open_);
    RCLCPP_INFO(n->get_logger(), "gripper_closed_: %f",         gripper_closed_);
    RCLCPP_INFO(n->get_logger(), "max_curvature: %f",           max_curvature_);
    RCLCPP_INFO(n->get_logger(), "max_speed_finger: %f",        max_speed_finger_);
    RCLCPP_INFO(n->get_logger(), "num_fingers: %d",             num_fingers_);

    // Update topic prefix
    if (arm_name_ == "jaco2")
      topic_prefix_ = "jaco";
    else
      topic_prefix_ = arm_name_;

    // Update total number of joints
    if (kinova_gripper_)
      num_joints_ = 2*num_fingers_ + NUM_JACO_JOINTS;
    else
      num_joints_ = NUM_JACO_JOINTS;

    joint_pos_.resize(num_joints_);
    joint_vel_.resize(num_joints_);
    joint_eff_.resize(num_joints_);
    RCLCPP_INFO(n->get_logger(), "Parameters loaded.");

    //! @todo MdL [IMPR]: Return is values are all correctly loaded.
    return true;
}

/*****************************************/
/**********  Basic Arm Commands **********/
/*****************************************/

void JacoArmTrajectoryController::angularCmdCallback(const wpi_jaco_msgs::msg::AngularCommand::SharedPtr msg)
{
  if (eStopEnabled)
    return;

  //take control of the arm
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();

    if (controlType != ANGULAR_CONTROL)
    {
      SetAngularControl();
      controlType = ANGULAR_CONTROL;
    }
  }

  TrajectoryPoint jacoPoint;
  jacoPoint.InitStruct();

  //populate arm command
  if (msg->arm_command)
  {
    if (msg->position)
    {
      jacoPoint.Position.Type = ANGULAR_POSITION;
      AngularPosition position_data;

      float current_joint_pos[6];
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetAngularPosition(position_data);
      }
      current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;

      jacoPoint.Position.Actuators.Actuator1 = nearest_equivalent(simplify_angle(msg->joints[0]),
                                                                  current_joint_pos[0]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator2 = nearest_equivalent(simplify_angle(msg->joints[1]),
                                                                  current_joint_pos[1]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator3 = nearest_equivalent(simplify_angle(msg->joints[2]),
                                                                  current_joint_pos[2]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator4 = nearest_equivalent(simplify_angle(msg->joints[3]),
                                                                  current_joint_pos[3]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator5 = nearest_equivalent(simplify_angle(msg->joints[4]),
                                                                  current_joint_pos[4]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator6 = nearest_equivalent(simplify_angle(msg->joints[5]),
                                                                  current_joint_pos[5]) * RAD_TO_DEG;
    }
    else
    {
      jacoPoint.Position.Type = ANGULAR_VELOCITY;
      jacoPoint.Position.Actuators.Actuator1 = msg->joints[0] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator2 = msg->joints[1] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator3 = msg->joints[2] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator4 = msg->joints[3] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator5 = msg->joints[4] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator6 = msg->joints[5] * RAD_TO_DEG;
    }

  }
  else
  {
    if (msg->position)
    {
      fingerPositionControl(msg->fingers[0], msg->fingers[1], msg->fingers[2]);
      return;
    }
    else
    {
      jacoPoint.Position.Type = ANGULAR_VELOCITY;
      jacoPoint.Position.Actuators.Actuator1 = 0.0;
      jacoPoint.Position.Actuators.Actuator2 = 0.0;
      jacoPoint.Position.Actuators.Actuator3 = 0.0;
      jacoPoint.Position.Actuators.Actuator4 = 0.0;
      jacoPoint.Position.Actuators.Actuator5 = 0.0;
      jacoPoint.Position.Actuators.Actuator6 = 0.0;
    }
  }

  //populate finger command
  if (msg->finger_command)
  {
    if (msg->position)
      jacoPoint.Position.HandMode = POSITION_MODE;
    else
      jacoPoint.Position.HandMode = VELOCITY_MODE;

    jacoPoint.Position.Fingers.Finger1 = msg->fingers[0];
    jacoPoint.Position.Fingers.Finger2 = msg->fingers[1];
    jacoPoint.Position.Fingers.Finger3 = msg->fingers[2];
  }
  else
    jacoPoint.Position.HandMode = HAND_NOMOVEMENT;

  //send command
  if (msg->position)
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    SendBasicTrajectory(jacoPoint);
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    if (msg->repeat)
    {
      //send the command repeatedly for ~1/60th of a second
      //(this is sometimes necessary for velocity commands to work correctly)
      rclcpp::Rate rate(600);
      for (int i = 0; i < 10; i++)
      {
        SendBasicTrajectory(jacoPoint);
        rate.sleep();
      }
    }
    else
      SendBasicTrajectory(jacoPoint);
  }
}

void JacoArmTrajectoryController::cartesianCmdCallback(const wpi_jaco_msgs::msg::CartesianCommand::SharedPtr msg)
{
  if (eStopEnabled)
    return;

  //take control of the arm
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();

    if (controlType != CARTESIAN_CONTROL)
    {
      SetCartesianControl();
      controlType = CARTESIAN_CONTROL;
    }
  }

  TrajectoryPoint jacoPoint;
  jacoPoint.InitStruct();

  //populate arm command
  if (msg->arm_command)
  {
    if (msg->position)
      jacoPoint.Position.Type = CARTESIAN_POSITION;
    else
      jacoPoint.Position.Type = CARTESIAN_VELOCITY;
    jacoPoint.Position.CartesianPosition.X = msg->arm.linear.x;
    jacoPoint.Position.CartesianPosition.Y = msg->arm.linear.y;
    jacoPoint.Position.CartesianPosition.Z = msg->arm.linear.z;
    jacoPoint.Position.CartesianPosition.ThetaX = msg->arm.angular.x;
    jacoPoint.Position.CartesianPosition.ThetaY = msg->arm.angular.y;
    jacoPoint.Position.CartesianPosition.ThetaZ = msg->arm.angular.z;
  }
  else
  {
    if (msg->position)
    {
      fingerPositionControl(msg->fingers[0], msg->fingers[1], msg->fingers[2]);
      return;
    }
    else
    {
      jacoPoint.Position.Type = ANGULAR_VELOCITY;
      jacoPoint.Position.Actuators.Actuator1 = 0.0;
      jacoPoint.Position.Actuators.Actuator2 = 0.0;
      jacoPoint.Position.Actuators.Actuator3 = 0.0;
      jacoPoint.Position.Actuators.Actuator4 = 0.0;
      jacoPoint.Position.Actuators.Actuator5 = 0.0;
      jacoPoint.Position.Actuators.Actuator6 = 0.0;
    }
  }

  //populate finger command
  if (msg->finger_command)
  {
    if (msg->position)
      jacoPoint.Position.HandMode = POSITION_MODE;
    else
      jacoPoint.Position.HandMode = VELOCITY_MODE;
    jacoPoint.Position.Fingers.Finger1 = msg->fingers[0];
    jacoPoint.Position.Fingers.Finger2 = msg->fingers[1];
    jacoPoint.Position.Fingers.Finger3 = msg->fingers[2];
  }
  else
    jacoPoint.Position.HandMode = HAND_NOMOVEMENT;

  //send command
  if (msg->position)
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    SendBasicTrajectory(jacoPoint);
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    SendBasicTrajectory(jacoPoint);
    if (msg->repeat)
    {
      //send the command repeatedly for ~1/60th of a second
      //(this is sometimes necessary for velocity commands to work correctly)
      rclcpp::Rate rate(600);
      for (int i = 0; i < 10; i++)
      {
        SendBasicTrajectory(jacoPoint);
        rate.sleep();
      }
    }
  }
}

void JacoArmTrajectoryController::fingerPositionControl(float f1, float f2, float f3)
{
  if (eStopEnabled)
    return;

  f1 = max(f1, .02f);
  f2 = max(f2, .02f);
  f3 = max(f3, .02f);

  TrajectoryPoint jacoPoint;
  jacoPoint.InitStruct();
  jacoPoint.Position.Type = ANGULAR_VELOCITY;
  jacoPoint.Position.Actuators.Actuator1 = 0.0;
  jacoPoint.Position.Actuators.Actuator2 = 0.0;
  jacoPoint.Position.Actuators.Actuator3 = 0.0;
  jacoPoint.Position.Actuators.Actuator4 = 0.0;
  jacoPoint.Position.Actuators.Actuator5 = 0.0;
  jacoPoint.Position.Actuators.Actuator6 = 0.0;
  jacoPoint.Position.HandMode = VELOCITY_MODE;

  bool goalReached = false;
  AngularPosition position_data;
  float error[3];
  float prevTotalError;
  int counter = 0; //check if error is unchanging, this likely means a finger is blocked by something so the controller should terminate
  vector<float> errorFinger1;
  vector<float> errorFinger2;
  vector<float> errorFinger3;
  errorFinger1.resize(10);
  errorFinger2.resize(10);
  errorFinger3.resize(10);
  for (unsigned int i = 0; i < errorFinger1.size(); i++)
  {
    errorFinger1[i] = 0.0;
    errorFinger2[i] = 0.0;
    errorFinger3[i] = 0.0;
  }
  rclcpp::Rate rate(600);
  while (!goalReached)
  {
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);

      //get current finger position
      GetAngularPosition(position_data);
      error[0] = f1 - position_data.Fingers.Finger1;
      error[1] = f2 - position_data.Fingers.Finger2;
      if (num_fingers_ == 3)
        error[2] = f3 - position_data.Fingers.Finger3;
      else
        error[2] = 0.0;

      float totalError = fabs(error[0]) + fabs(error[1]) + fabs(error[2]);
      if (totalError == prevTotalError)
      {
        counter++;
      }
      else
      {
        counter = 0;
        prevTotalError = totalError;
      }

      // RCLCPP_INFO(nh->get_logger(), "Current error: %f, previous error: %f", totalError, prevTotalError);

      if (totalError < finger_error_threshold_ || counter > 40)
      {
        goalReached = true;
        jacoPoint.Position.Fingers.Finger1 = 0.0;
        jacoPoint.Position.Fingers.Finger2 = 0.0;
        jacoPoint.Position.Fingers.Finger3 = 0.0;
      }
      else
      {
        float errorSum[3] = {0};
        for (unsigned int i = 0; i < errorFinger1.size(); i ++)
        {
          errorSum[0] += errorFinger1[i];
          errorSum[1] += errorFinger2[i];
          errorSum[2] += errorFinger3[i];
        }
        jacoPoint.Position.Fingers.Finger1 = max(min(KP_F*error[0] + KV_F*(error[0] - errorFinger1.front()) + KI_F*errorSum[0], max_speed_finger_), -max_speed_finger_);
        jacoPoint.Position.Fingers.Finger2 = max(min(KP_F*error[1] + KV_F*(error[1] - errorFinger2.front()) + KI_F*errorSum[1], max_speed_finger_), -max_speed_finger_);
        jacoPoint.Position.Fingers.Finger3 = max(min(KP_F*error[2] + KV_F*(error[2] - errorFinger3.front()) + KI_F*errorSum[2], max_speed_finger_), -max_speed_finger_);

        errorFinger1.insert(errorFinger1.begin(), error[0]);
        errorFinger2.insert(errorFinger2.begin(), error[1]);
        errorFinger3.insert(errorFinger3.begin(), error[2]);

        errorFinger1.resize(10);
        errorFinger2.resize(10);
        errorFinger3.resize(10);
      }
      
      EraseAllTrajectories();
      SendBasicTrajectory(jacoPoint);
    }

    //check for cancel requests
    if (eStopEnabled)
      return;

    rate.sleep();
  }

  RCLCPP_INFO(nh->get_logger(), "Goal reached, counter: %d, total error: %f", counter, prevTotalError);
}

void JacoArmTrajectoryController::executeAngularTrajectoryPoint(TrajectoryPoint point, bool erase)
{
  if (eStopEnabled)
    return;

  boost::recursive_mutex::scoped_lock lock(api_mutex);

  if (controlType != ANGULAR_CONTROL)
  {
    SetAngularControl();
    controlType = ANGULAR_CONTROL;
  }

  if (erase)
    EraseAllTrajectories();

  SendBasicTrajectory(point);
}

void JacoArmTrajectoryController::executeCartesianTrajectoryPoint(TrajectoryPoint point, bool erase)
{
  if (eStopEnabled)
    return;

  boost::recursive_mutex::scoped_lock lock(api_mutex);

  if (controlType != CARTESIAN_CONTROL)
  {
    SetCartesianControl();
    controlType = CARTESIAN_CONTROL;
  }

  if (erase)
    EraseAllTrajectories();

  SendBasicTrajectory(point);
}

bool JacoArmTrajectoryController::getAngularPosition(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<wpi_jaco_msgs::srv::GetAngularPosition::Request> req,
      std::shared_ptr<wpi_jaco_msgs::srv::GetAngularPosition::Response> res)
{
  res->pos.resize(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i ++)
  {
    res->pos[i] = joint_pos_[i];
  }

  return true;
}

bool JacoArmTrajectoryController::getCartesianPosition(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<wpi_jaco_msgs::srv::GetCartesianPosition::Request> req,
      std::shared_ptr<wpi_jaco_msgs::srv::GetCartesianPosition::Response> res)
{
  CartesianPosition pos;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetCartesianPosition(pos);
  }
  res->pos.linear.x = pos.Coordinates.X;
  res->pos.linear.y = pos.Coordinates.Y;
  res->pos.linear.z = pos.Coordinates.Z;
  res->pos.angular.x = pos.Coordinates.ThetaX;
  res->pos.angular.y = pos.Coordinates.ThetaY;
  res->pos.angular.z = pos.Coordinates.ThetaZ;

  return true;
}

bool JacoArmTrajectoryController::eStopCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<wpi_jaco_msgs::srv::EStop::Request> req,
      std::shared_ptr<wpi_jaco_msgs::srv::EStop::Response> res)
{
  res->success = true;
  if (req->enable_e_stop)
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    RCLCPP_INFO(nh->get_logger(), "Software emergency stop request received.");
    if (!eStopEnabled)
    {
      int stopResult = StopControlAPI();
      if (stopResult != NO_KINOVA_ERROR)
      {
        RCLCPP_INFO(nh->get_logger(), "Error stopping arm control.");
        res->success = false;
      }
      stopResult = EraseAllTrajectories();
      if (stopResult != NO_KINOVA_ERROR)
      {
        RCLCPP_INFO(nh->get_logger(), "Error stopping arm trajectories.");
        res->success = false;
      }
      eStopEnabled = true;
    }
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    RCLCPP_INFO(nh->get_logger(), "Turning off software emergency stop.");
    if (eStopEnabled)
    {
      StopControlAPI();
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      StartControlAPI();
      if (controlType == ANGULAR_CONTROL)
        SetAngularControl();
      else
        SetCartesianControl();
      eStopEnabled = false;
    }
  }

  return true;
}

bool JacoArmTrajectoryController::eraseTrajectoriesCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> req,
      std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  if (EraseAllTrajectories() != NO_KINOVA_ERROR)
  {
    RCLCPP_INFO(nh->get_logger(), "Error stopping arm trajectories.");
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("jaco_arm_trajectory_node");

  jaco::JacoArmTrajectoryController robot(node);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
