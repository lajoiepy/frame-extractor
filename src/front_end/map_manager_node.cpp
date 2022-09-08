#include "slam_frame_extractor/front_end/map_manager.h"

using namespace slam_frame_extractor;

/**
 * @brief Node to manage the sensor data and registration
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("map_manager");

  node->declare_parameter<int>("frontend.pnp_min_inliers", 20);
  node->declare_parameter<int>("frontend.max_keyframe_queue_size", 10);
  node->declare_parameter<int>("frontend.map_manager_process_period_ms", 100);
  node->declare_parameter<std::string>("results_save_path", "");

  auto lcd = std::make_shared<MapManager<StereoHandler>>(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
