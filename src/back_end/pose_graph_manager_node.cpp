#include "slam_frame_extractor/back_end/pgo.h"

using namespace slam_frame_extractor;

/**
 * @brief Node to manage the pose graph data
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pose_graph_manager");

  node->declare_parameter<int>("backend.pose_graph_optimization_start_period_ms", 1000);
  node->declare_parameter<std::string>("results_save_path", "");
  node->declare_parameter<double>("backend.residual_inlier_threshold", 1.0);
  node->declare_parameter<double>("backend.residual_outlier_threshold", 100.0);

  PGO manager(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
