#ifndef _STEREOHANDLER_H_
#define _STEREOHANDLER_H_

#include <rclcpp/rclcpp.hpp>

#include <rtabmap_ros/msg/rgbd_image.hpp>
#include <rtabmap_ros/srv/add_link.hpp>
#include <rtabmap_ros/srv/get_map.hpp>

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UStl.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <slam_interfaces/msg/keyframe_odom.hpp>
#include <slam_interfaces/msg/keyframe_rgb.hpp>
#include <slam_interfaces/msg/loop_closure.hpp>
#include <slam_interfaces/msg/local_keyframe_matches.hpp>
#include <deque>
#include <functional>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>
#include <fstream>
#include <memory>

#include "slam_frame_extractor/front_end/sensor_msg_utils.h"

namespace slam_frame_extractor
{

    class StereoHandler
    {
    public:
        /**
         * @brief Initialization of parameters and ROS 2 objects
         *
         * @param node ROS 2 node handle
         */
        StereoHandler(std::shared_ptr<rclcpp::Node> &node);
        ~StereoHandler(){};

        /**
         * @brief Processes Latest received image
         *
         */
        void process_new_sensor_data();

        /**
         * @brief Receives a local match and tries to compute a local loop closure
         *
         * @param msg
         */
        void receive_local_keyframe_match(
            slam_interfaces::msg::LocalKeyframeMatches::ConstSharedPtr
                msg);

        /**
         * @brief Extract training triplets
         *
         * @param fromId
         * @param toId
         * @param regInfo
         * @param tmpFrom
         */
        void extract_triplets(const unsigned int from_id, const unsigned int to_id, rtabmap::RegistrationInfo &reg_info, const std::shared_ptr<rtabmap::SensorData> tmp_from, const std::vector<unsigned int> &best_matches);

        /**
         * @brief Computes local 3D descriptors from frame data and store them
         *
         * @param frame_data Full frame data
         */
        void
        compute_local_descriptors(std::shared_ptr<rtabmap::SensorData> &frame_data);

        /**
         * @brief Generate a new keyframe according to the policy
         *
         * @param keyframe Sensor data
         * @return true A new keyframe is added to the map
         * @return false The frame is rejected
         */
        bool generate_new_keyframe(std::shared_ptr<rtabmap::SensorData> &keyframe);

        /**
         * @brief Function to send the image to the python node
         * TODO: Move to parent class
         *
         * @param data keyframe data
         * @param id keyframe id
         */
        void send_keyframe(const rtabmap::SensorData &data,
                           const nav_msgs::msg::Odometry::ConstSharedPtr odom,
                           const int id);

        /**
         * @brief Callback receiving sync data from camera
         *
         * @param imageRectLeft
         * @param imageRectRight
         * @param cameraInfoLeft
         * @param cameraInfoRight
         * @param odom
         */
        void stereo_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr imageRectLeft,
            const sensor_msgs::msg::Image::ConstSharedPtr imageRectRight,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight,
            const nav_msgs::msg::Odometry::ConstSharedPtr odom);

    private:
        std::deque<std::pair<std::shared_ptr<rtabmap::SensorData>,
                             nav_msgs::msg::Odometry::ConstSharedPtr>>
            received_data_queue_;

        std::shared_ptr<rtabmap::SensorData> previous_keyframe_;

        std::map<int, std::shared_ptr<rtabmap::SensorData>> local_descriptors_map_;

        std::shared_ptr<rclcpp::Node> node_;

        unsigned int min_inliers_, max_queue_size_,
            nb_local_keyframes_, keyframe_id_;

        image_transport::SubscriberFilter image_rect_left_;
        image_transport::SubscriberFilter image_rect_right_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_left_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_right_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_;
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
            nav_msgs::msg::Odometry>
            SyncPolicy;
        message_filters::Synchronizer<SyncPolicy> *sync_policy_;

        rclcpp::Publisher<slam_interfaces::msg::KeyframeRGB>::SharedPtr
            keyframe_data_publisher_;

        rclcpp::Publisher<slam_interfaces::msg::KeyframeOdom>::SharedPtr
            keyframe_odom_publisher_;

        rclcpp::Subscription<
            slam_interfaces::msg::LocalKeyframeMatches>::SharedPtr
            local_keyframe_match_subscriber_;

        rclcpp::Publisher<
            slam_interfaces::msg::LoopClosure>::SharedPtr
            loop_closure_publisher_;

        rtabmap::RegistrationVis registration_;

        std::string base_frame_id_;
        float keyframe_generation_ratio_;
        bool generate_new_keyframes_based_on_inliers_ratio_;
        std::string results_save_path_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
} // namespace slam_frame_extractor
#endif