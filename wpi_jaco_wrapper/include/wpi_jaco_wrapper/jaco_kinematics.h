/*!
 * \jaco_kinematics.h
 * \brief Provides services for JACO kinematics.
 *
 * jaco_kinematics creates a ROS node that provides services for converting calculating
 * kinematics for the JACO arm.
 *
 * \author David Kent, GT - dekent@gatech.edu
 */

#ifndef JACO_ARM_KINEMATICS_H_
#define JACO_ARM_KINEMATICS_H_

#include <rclcpp/rclcpp.hpp>
#include <wpi_jaco_msgs/msg/joints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>

//Link lengths and offsets (JACO)
#define D1 .2755
#define D2 .4100
#define D3 .2073
#define D4 .0743
#define D5 .0743
#define D6 .1687
#define E2 .0098

//Link lengths and offsets (JACO2)
#define J2D1 .2755
#define J2D2 .4100
#define J2D3 .2073
#define J2D4 .0741
#define J2D5 .0741
#define J2D6 .1600
#define J2E2 .0098

#define PI 3.14159

/*!
 * \class JacoKinematics
 * \brief Provides services for JACO kinematics.
 *
 * JacoKinematics creates a ROS node that provides services for converting calculating
 * kinematics for the JACO arm.
 */
class JacoKinematics
{

public:

  JacoKinematics(const std::shared_ptr<rclcpp::Node> n);

  /**
   * \brief Callback for the forward kinematics service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  geometry_msgs::msg::PoseStamped callFK(const wpi_jaco_msgs::msg::Joints joints);

  /**
   * \brief Calculates the forward kinematics for the JACO arm
   * @param joints vector of joint angles from the arm
   * @return pose of the end effector relative to the arm's base
   */
  geometry_msgs::msg::PoseStamped calculateFK(std::vector<float> joints);

  /**
   * \brief Generates a transform given D-H parameters
   * @param theta joint angle
   * @param d link length
   * @param a offset
   * @param alpha angle offset
   * @return the transform for one link
   */
  tf2::Transform generateTransform(float theta, float d, float a, float alpha);

private:
  std::string arm_name_;
  std::string topic_prefix_;

  //robot parameters
  std::vector<double> ds; //!< d parameters in the D-H convention
  std::vector<double> as; //!< a parameters in the D-H convention
  std::vector<double> alphas; //!< alpha parameters in the D-H convention
};

#endif
