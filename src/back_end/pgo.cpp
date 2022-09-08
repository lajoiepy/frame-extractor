#include "slam_frame_extractor/back_end/pgo.h"

using namespace slam_frame_extractor;

PGO::PGO(std::shared_ptr<rclcpp::Node> &node)
    : node_(node)
{

  node_->get_parameter("backend.pose_graph_optimization_start_period_ms",
                       pose_graph_optimization_start_period_ms_);
  node->get_parameter("results_save_path",
                      log_optimization_files_path_);
  node->get_parameter("backend.residual_inlier_threshold",
                      inlier_threshold_);
  node->get_parameter("backend.residual_outlier_threshold",
                      outlier_threshold_);

  odometry_subscriber_ =
      node->create_subscription<slam_interfaces::msg::KeyframeOdom>(
          "keyframe_odom", 1000,
          std::bind(&PGO::odometry_callback, this,
                    std::placeholders::_1));

  loop_closure_subscriber_ = node->create_subscription<
      slam_interfaces::msg::LoopClosure>(
      "loop_closure", 1000,
      std::bind(&PGO::loop_closure_callback, this,
                std::placeholders::_1));

  rotation_default_noise_std_ = 0.01;
  translation_default_noise_std_ = 0.1;
  Eigen::VectorXd sigmas(6);
  sigmas << rotation_default_noise_std_, rotation_default_noise_std_,
      rotation_default_noise_std_, translation_default_noise_std_,
      translation_default_noise_std_, translation_default_noise_std_;
  default_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  pose_graph_ = boost::make_shared<gtsam::NonlinearFactorGraph>();
  current_pose_estimates_ = boost::make_shared<gtsam::Values>();
  odometry_pose_estimates_ = boost::make_shared<gtsam::Values>();

  // Optimization timers
  optimization_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_optimization_start_period_ms_),
      std::bind(&PGO::optimization_callback, this));

  is_optimization_started_ = false;

  RCLCPP_INFO(node_->get_logger(), "Initialization done.");
}

void PGO::odometry_callback(
    const slam_interfaces::msg::KeyframeOdom::ConstSharedPtr msg)
{

  gtsam::Pose3 current_estimate = odometry_msg_to_pose3(msg->odom);
  gtsam::Key symbol(msg->id);

  odometry_pose_estimates_->insert(symbol, current_estimate);
  if (msg->id == 0)
  {
    current_pose_estimates_->insert(symbol, current_estimate);
  }

  if (odometry_pose_estimates_->size() > 1)
  {
    gtsam::Pose3 odom_diff = latest_local_pose_.inverse() * current_estimate;
    gtsam::BetweenFactor<gtsam::Pose3> factor(latest_local_symbol_, symbol,
                                              odom_diff, default_noise_model_);
    pose_graph_->push_back(factor);
  }

  // Update latest pose
  latest_local_pose_ = current_estimate;
  latest_local_symbol_ = symbol;
}

void PGO::loop_closure_callback(
    const slam_interfaces::msg::LoopClosure::
        ConstSharedPtr msg)
{
  if (msg->success)
  {
    gtsam::Pose3 measurement = transform_msg_to_pose3(msg->transform);

    gtsam::Key symbol_from(msg->keyframe0_id);
    gtsam::Key symbol_to(msg->keyframe1_id);

    gtsam::BetweenFactor<gtsam::Pose3> factor =
        gtsam::BetweenFactor<gtsam::Pose3>(symbol_from, symbol_to, measurement,
                                           default_noise_model_);

    pose_graph_->push_back(factor);
    loop_closure_factors_.push_back(factor);
  }
}

gtsam::Values
PGO::optimize(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
              const gtsam::Values::shared_ptr &initial)
{
  if (!is_optimization_started_)
  {
    RCLCPP_WARN(node_->get_logger(),
                "Optimization not started. Skipping this call.");
    return *current_pose_estimates_;
  }

  gtsam::GncParams<gtsam::LevenbergMarquardtParams> params;
  gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
      optimizer(*graph, *initial, params);

  auto result = optimizer.optimize();
  return result;
}

void PGO::start_optimization()
{
  // Check if there are enough factors to start optimization
  if (odometry_pose_estimates_->size() < 2 || pose_graph_->size() < 2) //|| loop_closure_factors_.size() < 1)
  {
    return;
  }

  // Build global pose graph
  gtsam::writeG2o(*pose_graph_, *odometry_pose_estimates_,
                    log_optimization_files_path_ + "/before_optimization.g2o");

  // Add prior
  // Use first pose of current estimate
  gtsam::Key first_symbol(0);

  if (!odometry_pose_estimates_->exists(first_symbol))
  {
    return;
  }

  pose_graph_->addPrior(
      first_symbol, odometry_pose_estimates_->at<gtsam::Pose3>(first_symbol),
      default_noise_model_);

  is_optimization_started_ = true;
  // Optimize graph
  optimization_result_ =
      std::async(&PGO::optimize, this, pose_graph_,
                 odometry_pose_estimates_);
}

void PGO::write_samples_to_file(const gtsam::Values &result)
{
  std::ofstream inlier_file;
  inlier_file.open(log_optimization_files_path_ + "/inliers.txt", std::ofstream::out | std::ofstream::trunc);
  std::ofstream outlier_file;
  outlier_file.open(log_optimization_files_path_ + "/outliers.txt", std::ofstream::out | std::ofstream::trunc);
  for (size_t i = 0; i < loop_closure_factors_.size(); i++)
  {
    if (result.exists(loop_closure_factors_[i].key1()) && result.exists(loop_closure_factors_[i].key2()))
    {
      auto error = loop_closure_factors_[i].error(result);
      if (error < inlier_threshold_)
      {
        inlier_file << loop_closure_factors_[i].key1() << " "
             << loop_closure_factors_[i].key2() << std::endl;
      }
      else if(error > outlier_threshold_){
        outlier_file << loop_closure_factors_[i].key1() << " "
             << loop_closure_factors_[i].key2() << std::endl;
      }
    }
  }
  inlier_file.close();
  outlier_file.close();
}

void PGO::check_result_and_finish_optimization()
{
  auto status = optimization_result_.wait_for(std::chrono::milliseconds(0));

  if (status == std::future_status::ready)
  {
    try{
    RCLCPP_INFO(node_->get_logger(), "Pose Graph Optimization complete.");
    auto result = optimization_result_.get();
    write_samples_to_file(result);
    *current_pose_estimates_ = result;
    optimization_count_++;

    gtsam::writeG2o(
          *pose_graph_, result,
          log_optimization_files_path_ + "/optimized.g2o");
    } catch (std::exception &e) {
      // Reject result if their is a race condition
      // It will be solve in the next iteration.
    }
    is_optimization_started_ = false;
  }
}

void PGO::optimization_callback()
{
  if (!is_optimization_started_)
  {
    start_optimization();
  }
  else
  {
    check_result_and_finish_optimization();
  }
}