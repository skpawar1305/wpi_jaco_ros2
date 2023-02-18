#include <wpi_jaco_wrapper/jaco_kinematics.h>

using namespace std;

JacoKinematics::JacoKinematics(const std::shared_ptr<rclcpp::Node> n)
{
  arm_name_ = std::string("jaco");
  if (!n->has_parameter("wpi_jaco/arm_name"))
      n->declare_parameter("wpi_jaco/arm_name", arm_name_);
  n->get_parameter("wpi_jaco/arm_name", arm_name_);

  // Update topic prefix
  if (arm_name_ == "jaco2")
      topic_prefix_ = "jaco";
  else
      topic_prefix_ = arm_name_;

  //calculate additional parameters
  double AA, CA, SA, C2A, S2A, D4B, D5B, D6B;

  if (arm_name_ == "jaco2")
    AA = 30.0 * PI / 180.0;
  else
    AA = 11.0 * PI / 72.0;

  //common parameters
  CA = cos(AA);
  SA = sin(AA);
  C2A = cos(2 * AA);
  S2A = sin(2 * AA);

  if (arm_name_ == "jaco2")
  {
    //JACO2 parameters
    D4B = J2D3 + SA / S2A * J2D4;
    D5B = SA / S2A * J2D4 + SA / S2A * J2D5;
    D6B = SA / S2A * J2D5 + J2D6;
  }
  else
  {
    //JACO parameters
    D4B = D3 + SA / S2A * D4;
    D5B = SA / S2A * D4 + SA / S2A * D5;
    D6B = SA / S2A * D5 + D6;
  }

  //set up D-H parameters
  ds.resize(6);
  if (arm_name_ == "jaco2")
  {
    ds[0] = J2D1;
    ds[2] = -J2E2;
  }
  else
  {
    ds[0] = D1;
    ds[2] = -E2;
  }
  ds[1] = 0;
  ds[3] = -D4B;
  ds[4] = -D5B;
  ds[5] = -D6B;

  as.resize(6);
  for (unsigned int i = 0; i < 6; i++)
  {
    as[i] = 0;
  }
  if (arm_name_ == "jaco2")
    as[1] = J2D2;
  else
    as[1] = D2;

  alphas.resize(6);
  alphas[0] = PI / 2.0;
  alphas[1] = PI;
  alphas[2] = PI / 2.0;
  alphas[3] = 2 * AA;
  alphas[4] = 2 * AA;
  alphas[5] = PI;
}

geometry_msgs::msg::PoseStamped JacoKinematics::callFK(const wpi_jaco_msgs::msg::Joints j)
{
  geometry_msgs::msg::PoseStamped ps;

  if (j.joints.size() < 6)
  {
    throw std::invalid_argument("Not enough joints specified, could not calculate forward kinematics");
  }

  ps = calculateFK(j.joints);

  return ps;
}

geometry_msgs::msg::PoseStamped JacoKinematics::calculateFK(vector<float> joints)
{
  tf2::Transform transform;
  tf2::Transform tb1, t12, t23, t34, t45, t56, t6e;
  tf2::Quaternion rotQuat(0, 0, 0, 0);
  tf2::Matrix3x3 rotMat(1, 0, 0, 0, 1, 0, 0, 0, 1);
  tf2::Vector3 trans(0, 0, 0);

  //initialize empty transformation
  rotMat.getRotation(rotQuat);
  transform.setRotation(rotQuat);
  transform.setOrigin(trans);

  //Transform angles for D-H algorithm
  vector<float> thetas;
  thetas.resize(6);
  thetas[0] = -joints[0];
  thetas[1] = joints[1] - PI / 2.0;
  thetas[2] = joints[2] + PI / 2.0;
  thetas[3] = joints[3];
  thetas[4] = joints[4] - PI;
  if (arm_name_ == "jaco2")
    thetas[5] = joints[5] + PI / 2.0;
  else
    thetas[5] = joints[5] + 5.0 * PI / 9.0;

  //Calculate transformation from base to end effector
  for (unsigned int i = 0; i < joints.size(); i++)
  {
    transform = transform * generateTransform(thetas[i], ds[i], as[i], alphas[i]);
  }

  //Calculate Hand Pose
  geometry_msgs::msg::PoseStamped handPose;
  handPose.header.frame_id = arm_name_ + "_link_base";
  handPose.pose.position.x = transform.getOrigin().x();
  handPose.pose.position.y = transform.getOrigin().y();
  handPose.pose.position.z = transform.getOrigin().z();
  handPose.pose.orientation.x = transform.getRotation().x();
  handPose.pose.orientation.y = transform.getRotation().y();
  handPose.pose.orientation.z = transform.getRotation().z();
  handPose.pose.orientation.w = transform.getRotation().w();

  return handPose;
}

tf2::Transform JacoKinematics::generateTransform(float theta, float d, float a, float alpha)
{
  tf2::Transform transform;
  tf2::Quaternion rotQuat(0, 0, 0, 0);	//Rotation quaternion
  tf2::Matrix3x3 rotMat(0, 0, 0, 0, 0, 0, 0, 0, 0);	//Rotation matrix
  tf2::Vector3 trans(0, 0, 0);	//Translation vector

  //calculate rotation matrix
  rotMat.setValue(cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), sin(theta), cos(theta) * cos(alpha),
                  -cos(theta) * sin(alpha), 0, sin(alpha), cos(alpha));

  //calculate translation vector
  trans.setValue(a * cos(theta), a * sin(theta), d);

  //get rotation as a quaternion
  rotMat.getRotation(rotQuat);

  //fill transformation
  transform.setRotation(rotQuat);
  transform.setOrigin(trans);

  return transform;
}
