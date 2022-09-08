#ifndef _MAPMANAGER_H_
#define _MAPMANAGER_H_

#include <rclcpp/rclcpp.hpp>

#include <chrono>

#include "slam_frame_extractor/front_end/stereo_handler.h"

namespace slam_frame_extractor {
/**
 * @brief Loop Closure Detection Management
 * - Receives keyframes from RTAB-map
 * - Generate keypoints from frames
 * - Computes geometric verification
 *
 * @tparam DataHandlerType Depends on the type of input data (stereo, rgbd,
 * etc.)
 */
template <class DataHandlerType> class MapManager {
public:
  /**
   * @brief Initialization of parameters and ROS 2 objects
   *
   * @param node ROS 2 node handle
   */
  MapManager(std::shared_ptr<rclcpp::Node> &node);
  ~MapManager(){};

  /**
   * @brief Looks for loop closures in the current keyframe queue
   *
   */
  void process_new_sensor_data();

private:
  std::shared_ptr<rclcpp::Node> node_;

  int map_manager_process_period_ms_;

  DataHandlerType local_data_handler_;

  rclcpp::TimerBase::SharedPtr process_timer_;
};

// List possible data types for C++ linker
template class MapManager<StereoHandler>;
} // namespace slam_frame_extractor
#endif