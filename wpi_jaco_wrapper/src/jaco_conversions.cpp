#include <wpi_jaco_wrapper/jaco_conversions.h>

using namespace std;

JacoConversions::JacoConversions(const std::shared_ptr<rclcpp::Node> n)
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
}

geometry_msgs::msg::Quaternion JacoConversions::EulerToQuaternion(const wpi_jaco_msgs::msg::Euler e)
{
  float t1 = e.roll;
  float t2 = e.pitch;
  float t3 = e.yaw;

  geometry_msgs::msg::Quaternion q;
  // Calculate the quaternion given roll, pitch, and yaw (rotation order XYZ -- 1:X, 2:Y, 3:Z)
  q.w = -sin(t1 / 2.0) * sin(t2 / 2.0) * sin(t3 / 2.0) + cos(t1 / 2.0) * cos(t2 / 2.0) * cos(t3 / 2.0);
  q.x = sin(t1 / 2.0) * cos(t2 / 2.0) * cos(t3 / 2.0) + sin(t2 / 2.0) * sin(t3 / 2.0) * cos(t1 / 2.0);
  q.y = -sin(t1 / 2.0) * sin(t3 / 2.0) * cos(t2 / 2.0) + sin(t2 / 2.0) * cos(t1 / 2.0) * cos(t3 / 2.0);
  q.z = sin(t1 / 2.0) * sin(t2 / 2.0) * cos(t3 / 2.0) + sin(t3 / 2.0) * cos(t2 / 2.0) * cos(t2 / 2.0);

  return q;
}

wpi_jaco_msgs::msg::Euler JacoConversions::QuaternionToEuler(const geometry_msgs::msg::Quaternion q)
{
  float q1 = q.w;
  float q2 = q.x;
  float q3 = q.y;
  float q4 = q.z;

  // Calculate necessary elements of the rotation matrix
  float m11 = pow(q1, 2) + pow(q2, 2) - pow(q3, 2) - pow(q4, 2);
  float m12 = 2 * (q2 * q3 - q1 * q4);
  float m13 = 2 * (q2 * q4 + q1 * q3);
  float m23 = 2 * (q3 * q4 - q1 * q2);
  float m33 = pow(q1, 2) - pow(q2, 2) - pow(q3, 2) + pow(q4, 2);

  wpi_jaco_msgs::msg::Euler e;
  // Calculate the roll, pitch, and yaw (rotation order XYZ -- 1:X, 2:Y, 3:Z)
  e.roll = atan2(-m23, m33);
  e.pitch = atan2(m13, sqrt(1 - pow(m13, 2)));
  e.yaw = atan2(-m12, m11);

  return e;
}
