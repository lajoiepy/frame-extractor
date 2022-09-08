#ifndef _pgo_H_
#define _pgo_H_

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <slam_interfaces/msg/keyframe_odom.hpp>
#include <slam_interfaces/msg/loop_closure.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <future>
#include <gtsam/slam/dataset.h>
#include <list>
#include <thread>

#include "slam_frame_extractor/back_end/gtsam_utils.h"

namespace slam_frame_extractor
{

    class PGO
    {
    public:
        /**
         * @brief Initialization of parameters and ROS 2 objects
         *
         * @param node ROS 2 node handle
         */
        PGO(std::shared_ptr<rclcpp::Node> &node);
        ~PGO(){};

        /**
         * @brief Receives odometry msg + keyframe id
         *
         * @param msg
         */
        void odometry_callback(
            const slam_interfaces::msg::KeyframeOdom::ConstSharedPtr msg);

        /**
         * @brief Receives intra-robot loop closures
         *
         * @param msg
         */
        void loop_closure_callback(
            const slam_interfaces::msg::LoopClosure::
                ConstSharedPtr msg);

        /**
         * @brief Prints current estimates
         *
         * @param msg
         */
        void print_current_estimates_callback(
            const std_msgs::msg::String::ConstSharedPtr msg);

        /**
         * @brief Starts pose graph optimization process every X ms (defined in
         * config)
         *
         */
        void optimization_callback();

        /**
         * @brief Performs pose graph optimization
         *
         */
        void start_optimization();

        /**
         * @brief Pose graph optimization function
         *
         * @param graph pose graph
         * @param initial initial values
         * @return gtsam::Values optimized values
         */
        gtsam::Values optimize(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                               const gtsam::Values::shared_ptr &initial);

        /**
         * @brief Check if the optimization is finished
         *
         */
        void check_result_and_finish_optimization();

        /**
         * @brief Write inliers file
         * 
         * @param result 
         */
        void write_samples_to_file(const gtsam::Values &result);

    private:
        std::shared_ptr<rclcpp::Node> node_;

        unsigned int optimization_count_;

        double inlier_threshold_, outlier_threshold_;

        unsigned int pose_graph_optimization_start_period_ms_;

        gtsam::SharedNoiseModel default_noise_model_;
        float rotation_default_noise_std_, translation_default_noise_std_;

        gtsam::NonlinearFactorGraph::shared_ptr pose_graph_;
        std::vector<gtsam::BetweenFactor<gtsam::Pose3>> loop_closure_factors_;
        gtsam::Values::shared_ptr current_pose_estimates_;
        gtsam::Values::shared_ptr odometry_pose_estimates_;

        bool is_optimization_started_;

        gtsam::Pose3 latest_local_pose_;
        gtsam::Key latest_local_symbol_;

        rclcpp::Subscription<slam_interfaces::msg::KeyframeOdom>::SharedPtr
            odometry_subscriber_;

        rclcpp::Subscription<
            slam_interfaces::msg::LoopClosure>::SharedPtr
            loop_closure_subscriber_;

        std::future<gtsam::Values> optimization_result_;

        rclcpp::TimerBase::SharedPtr optimization_timer_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
            print_current_estimates_subscriber_;

        std::string log_optimization_files_path_;
    };
} // namespace slam_frame_extractor
#endif