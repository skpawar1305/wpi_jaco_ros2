/*!
 * \jaco_conversions.h
 * \brief Provides services for conversions between 3D rotation representations.
 *
 * jaco_conversions creates a ROS node that provides services for converting
 * between the JACO's internal representation of 3D rotations (Euler xyz convention)
 * and commonly used representations in ROS (quaternions)
 *
 * \author David Kent, GT - dekent@gatech.edu
 */

#ifndef JACO_CONVERSIONS_H_
#define JACO_CONVERSIONS_H_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <wpi_jaco_msgs/msg/euler.hpp>

/*!
 * \class JacoConversions
 * \brief Provides services for conversions between 3D rotation representations.
 *
 * JacoConversions creates a ROS node that provides services for converting
 * between the JACO's internal representation of 3D rotations (Euler xyz convention)
 * and commonly used representations in ROS (quaternions)
 */
class JacoConversions
{

public:

  JacoConversions(const std::shared_ptr<rclcpp::Node> n);

  geometry_msgs::msg::Quaternion EulerToQuaternion(const wpi_jaco_msgs::msg::Euler e);

  wpi_jaco_msgs::msg::Euler QuaternionToEuler(const geometry_msgs::msg::Quaternion q);

private:
  std::string        arm_name_;
  std::string        topic_prefix_;
};

#endif
