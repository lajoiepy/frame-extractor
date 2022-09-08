#include "slam_frame_extractor/front_end/map_manager.h"
#include "slam_frame_extractor/front_end/stereo_handler.h"

using namespace slam_frame_extractor;

template <class DataHandlerType>
MapManager<DataHandlerType>::MapManager(std::shared_ptr<rclcpp::Node> &node)
    : node_(node), local_data_handler_(node) {

  node_->get_parameter("frontend.map_manager_process_period_ms",
                       map_manager_process_period_ms_);

  std::chrono::milliseconds period(map_manager_process_period_ms_);

  process_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(period),
      std::bind(&MapManager<DataHandlerType>::process_new_sensor_data, this));

  RCLCPP_INFO(node_->get_logger(), "Initialization done.");
}

template <class DataHandlerType>
void MapManager<DataHandlerType>::process_new_sensor_data() {
  local_data_handler_.process_new_sensor_data();
}
