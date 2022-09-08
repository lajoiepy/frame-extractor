#ifndef GTSAMMSGCONVERSION_H_
#define GTSAMMSGCONVERSION_H_

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace slam_frame_extractor {

/**
 * @brief Converts odometry message to gtsam::Pose3
 *
 * @param odom_msg Odometry message
 * @return pose Pose data
 */
gtsam::Pose3 odometry_msg_to_pose3(const nav_msgs::msg::Odometry &odom_msg);

/**
 * @brief Converts a transform msg into a gtsam::Pose3
 *
 * @param msg Transform message
 * @return pose Pose data
 */
gtsam::Pose3 transform_msg_to_pose3(const geometry_msgs::msg::Transform &msg);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param pose 
 * @return geometry_msgs::msg::Pose 
 */
geometry_msgs::msg::Pose gtsam_pose_to_msg(const gtsam::Pose3 &pose);

/**
 * @brief Converts from GTSAM to ROS 2 message
 * 
 * @param pose 
 * @return geometry_msgs::msg::Transform 
 */
geometry_msgs::msg::Transform
gtsam_pose_to_transform_msg(const gtsam::Pose3 &pose);

/**
 * @brief Converts from ROS 2 message to GTSAM
 * 
 * @param pose 
 * @return gtsam::Pose3 
 */
gtsam::Pose3 pose_msg_to_gtsam(const geometry_msgs::msg::Pose &pose);

} // namespace slam_frame_extractor

#endif