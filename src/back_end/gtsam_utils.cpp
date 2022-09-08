#include "slam_frame_extractor/back_end/gtsam_utils.h"

namespace slam_frame_extractor {

geometry_msgs::msg::Pose gtsam_pose_to_msg(const gtsam::Pose3 &pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  msg.position.z = pose.z();
  auto rotation = pose.rotation().quaternion();
  msg.orientation.w = rotation[0];
  msg.orientation.x = rotation[1];
  msg.orientation.y = rotation[2];
  msg.orientation.z = rotation[3];
  return msg;
}

geometry_msgs::msg::Transform
gtsam_pose_to_transform_msg(const gtsam::Pose3 &pose) {
  geometry_msgs::msg::Transform msg;
  msg.translation.x = pose.x();
  msg.translation.y = pose.y();
  msg.translation.z = pose.z();

  auto rotation = pose.rotation().quaternion();
  msg.rotation.w = rotation[0];
  msg.rotation.x = rotation[1];
  msg.rotation.y = rotation[2];
  msg.rotation.z = rotation[3];

  return msg;
}

gtsam::Pose3 transform_msg_to_pose3(const geometry_msgs::msg::Transform &msg) {
  gtsam::Rot3 rotation(msg.rotation.w, msg.rotation.x, msg.rotation.y,
                       msg.rotation.z);
  return gtsam::Pose3(
      rotation, {msg.translation.x, msg.translation.y, msg.translation.z});
}

gtsam::Pose3 pose_msg_to_gtsam(const geometry_msgs::msg::Pose &pose) {
  gtsam::Rot3 rotation(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  return gtsam::Pose3(rotation,
                      {pose.position.x, pose.position.y, pose.position.z});
}

gtsam::Pose3 odometry_msg_to_pose3(const nav_msgs::msg::Odometry &odom_msg) {
  return pose_msg_to_gtsam(odom_msg.pose.pose);
}

} // namespace slam_frame_extractor