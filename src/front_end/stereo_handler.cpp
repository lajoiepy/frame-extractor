#include "slam_frame_extractor/front_end/stereo_handler.h"
#include "slam_frame_extractor/front_end/sensor_msg_utils.h"

using namespace rtabmap;
using namespace slam_frame_extractor;

StereoHandler::StereoHandler(std::shared_ptr<rclcpp::Node> &node)
    : node_(node)
{
  node->declare_parameter<std::string>("left_image_topic", "left/image_rect");
  node->declare_parameter<std::string>("right_image_topic", "right/image_rect");
  node->declare_parameter<std::string>("left_camera_info_topic",
                                       "left/camera_info");
  node->declare_parameter<std::string>("right_camera_info_topic",
                                       "right/camera_info");
  node->declare_parameter<std::string>("odom_topic", "odom");
  node->declare_parameter<float>("frontend.keyframe_generation_ratio", 0.0);
  node->declare_parameter<std::string>("sensor_base_frame_id", "camera_link");
  node_->get_parameter("frontend.max_keyframe_queue_size", max_queue_size_);
  node_->get_parameter("frontend.keyframe_generation_ratio", keyframe_generation_ratio_);
  node_->get_parameter("sensor_base_frame_id", base_frame_id_);
  node_->get_parameter("results_save_path", results_save_path_);

  if (keyframe_generation_ratio_ > 0.99)
  {
    generate_new_keyframes_based_on_inliers_ratio_ = false;
  }
  else
  {
    generate_new_keyframes_based_on_inliers_ratio_ = true;
  }

  nb_local_keyframes_ = 0;
  keyframe_id_ = 0;

  // Subscriber for stereo images
  int queue_size = 10;
  image_rect_left_.subscribe(
      node_.get(), node_->get_parameter("left_image_topic").as_string(), "raw",
      rclcpp::QoS(queue_size)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  image_rect_right_.subscribe(
      node_.get(), node_->get_parameter("right_image_topic").as_string(), "raw",
      rclcpp::QoS(queue_size)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  camera_info_left_.subscribe(
      node_.get(), node_->get_parameter("left_camera_info_topic").as_string(),
      rclcpp::QoS(queue_size)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  camera_info_right_.subscribe(
      node_.get(), node_->get_parameter("right_camera_info_topic").as_string(),
      rclcpp::QoS(queue_size)
          .reliability((rmw_qos_reliability_policy_t)2)
          .get_rmw_qos_profile());
  odometry_.subscribe(node_.get(),
                      node_->get_parameter("odom_topic").as_string(),
                      rclcpp::QoS(queue_size)
                          .reliability((rmw_qos_reliability_policy_t)2)
                          .get_rmw_qos_profile());

  sync_policy_ = new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(queue_size), image_rect_left_, image_rect_right_,
      camera_info_left_, camera_info_right_, odometry_);
  sync_policy_->registerCallback(
      std::bind(&StereoHandler::stereo_callback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

  // Parameters
  node_->get_parameter("frontend.max_keyframe_queue_size", max_queue_size_);
  node_->get_parameter("frontend.pnp_min_inliers", min_inliers_);

  // Publisher for global descriptors
  keyframe_data_publisher_ =
      node_->create_publisher<slam_interfaces::msg::KeyframeRGB>(
          "keyframe_data", 100);

  // Publisher for odometry with ID
  keyframe_odom_publisher_ =
      node_->create_publisher<slam_interfaces::msg::KeyframeOdom>(
          "keyframe_odom", 100);

  // Local matches subscription
  local_keyframe_match_subscriber_ = node->create_subscription<
      slam_interfaces::msg::LocalKeyframeMatches>(
      "local_keyframe_match", 100,
      std::bind(&StereoHandler::receive_local_keyframe_match, this,
                std::placeholders::_1));

  // Registration settings
  rtabmap::ParametersMap registration_params;
  registration_params.insert(rtabmap::ParametersPair(
      rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
  registration_.parseParameters(registration_params);

  // loop closure publisher
  loop_closure_publisher_ = node_->create_publisher<
      slam_interfaces::msg::LoopClosure>(
      "loop_closure", 100);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void StereoHandler::stereo_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr imageRectLeft,
    const sensor_msgs::msg::Image::ConstSharedPtr imageRectRight,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  if (!(imageRectLeft->encoding.compare(
            sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
        imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==
            0 ||
        imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==
            0 ||
        imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) ==
            0 ||
        imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) ==
            0 ||
        imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGRA8) ==
            0 ||
        imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGBA8) ==
            0) ||
      !(imageRectRight->encoding.compare(
            sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
        imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==
            0 ||
        imageRectRight->encoding.compare(
            sensor_msgs::image_encodings::MONO16) == 0 ||
        imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) ==
            0 ||
        imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) ==
            0 ||
        imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGRA8) ==
            0 ||
        imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGBA8) ==
            0))
  {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (mono8 "
        "recommended), received types are %s (left) and %s (right)",
        imageRectLeft->encoding.c_str(), imageRectRight->encoding.c_str());
    return;
  }

  rclcpp::Time stamp =
      rtabmap_ros::timestampFromROS(imageRectLeft->header.stamp) >
              rtabmap_ros::timestampFromROS(imageRectRight->header.stamp)
          ? imageRectLeft->header.stamp
          : imageRectRight->header.stamp;

  Transform localTransform = rtabmap_ros::getTransform(
      base_frame_id_, imageRectLeft->header.frame_id, stamp, *tf_buffer_, 0.1);
  if (localTransform.isNull())
  {
    return;
  }

  if (imageRectLeft->data.size() && imageRectRight->data.size())
  {
    bool alreadyRectified = true;
    rtabmap::Transform stereoTransform;
    if (!alreadyRectified)
    {
      stereoTransform = rtabmap_ros::getTransform(
          cameraInfoRight->header.frame_id, cameraInfoLeft->header.frame_id,
          cameraInfoLeft->header.stamp, *tf_buffer_, 0.1);
      if (stereoTransform.isNull())
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter %s is false but we cannot get TF between the "
                     "two cameras! (between frames %s and %s)",
                     Parameters::kRtabmapImagesAlreadyRectified().c_str(),
                     cameraInfoRight->header.frame_id.c_str(),
                     cameraInfoLeft->header.frame_id.c_str());
        return;
      }
      else if (stereoTransform.isIdentity())
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter %s is false but we cannot get a valid TF "
                     "between the two cameras! "
                     "Identity transform returned between left and right "
                     "cameras. Verify that if TF between "
                     "the cameras is valid: \"rosrun tf tf_echo %s %s\".",
                     Parameters::kRtabmapImagesAlreadyRectified().c_str(),
                     cameraInfoRight->header.frame_id.c_str(),
                     cameraInfoLeft->header.frame_id.c_str());
        return;
      }
    }

    rtabmap::StereoCameraModel stereoModel =
        rtabmap_ros::stereoCameraModelFromROS(*cameraInfoLeft, *cameraInfoRight,
                                              localTransform, stereoTransform);

    if (stereoModel.baseline() == 0 && alreadyRectified)
    {
      stereoTransform = rtabmap_ros::getTransform(
          cameraInfoLeft->header.frame_id, cameraInfoRight->header.frame_id,
          cameraInfoLeft->header.stamp, *tf_buffer_, 0.1);

      if (!stereoTransform.isNull() && stereoTransform.x() > 0)
      {
        static bool warned = false;
        if (!warned)
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "Right camera info doesn't have Tx set but we are assuming that "
              "stereo images are already rectified (see %s parameter). While "
              "not "
              "recommended, we used TF to get the baseline (%s->%s = %fm) for "
              "convenience (e.g., D400 ir stereo issue). It is preferred to "
              "feed "
              "a valid right camera info if stereo images are already "
              "rectified. This message is only printed once...",
              rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
              cameraInfoRight->header.frame_id.c_str(),
              cameraInfoLeft->header.frame_id.c_str(), stereoTransform.x());
          warned = true;
        }
        stereoModel = rtabmap::StereoCameraModel(
            stereoModel.left().fx(), stereoModel.left().fy(),
            stereoModel.left().cx(), stereoModel.left().cy(),
            stereoTransform.x(), stereoModel.localTransform(),
            stereoModel.left().imageSize());
      }
    }

    if (alreadyRectified && stereoModel.baseline() <= 0)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "The stereo baseline (%f) should be positive (baseline=-Tx/fx). We "
          "assume a horizontal left/right stereo "
          "setup where the Tx (or P(0,3)) is negative in the right camera info "
          "msg.",
          stereoModel.baseline());
      return;
    }

    if (stereoModel.baseline() > 10.0)
    {
      static bool shown = false;
      if (!shown)
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "Detected baseline (%f m) is quite large! Is your "
            "right camera_info P(0,3) correctly set? Note that "
            "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
            stereoModel.baseline());
        shown = true;
      }
    }

    cv_bridge::CvImagePtr ptrImageLeft = cv_bridge::toCvCopy(
        imageRectLeft, imageRectLeft->encoding.compare(
                           sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
                               imageRectLeft->encoding.compare(
                                   sensor_msgs::image_encodings::MONO8) == 0
                           ? ""
                       : imageRectLeft->encoding.compare(
                             sensor_msgs::image_encodings::MONO16) != 0
                           ? "bgr8"
                           : "mono8");
    cv_bridge::CvImagePtr ptrImageRight = cv_bridge::toCvCopy(
        imageRectRight, imageRectRight->encoding.compare(
                            sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
                                imageRectRight->encoding.compare(
                                    sensor_msgs::image_encodings::MONO8) == 0
                            ? ""
                            : "mono8");

    auto data = std::make_shared<rtabmap::SensorData>(
        ptrImageLeft->image, ptrImageRight->image, stereoModel,
        keyframe_id_, rtabmap_ros::timestampFromROS(stamp));
    keyframe_id_++;

    received_data_queue_.push_back(std::make_pair(data, odom));
    if (received_data_queue_.size() > max_queue_size_)
    {
      // Remove the oldest keyframes if we exceed the maximum size
      received_data_queue_.pop_front();
      RCLCPP_WARN(
          node_->get_logger(),
          "Maximum queue size (%d) exceeded, the oldest element was removed.",
          max_queue_size_);
    }
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Odom: input images empty?!");
  }
}

void StereoHandler::compute_local_descriptors(
    std::shared_ptr<rtabmap::SensorData> &frame_data)
{
  // Extract local descriptors
  frame_data->uncompressData();
  std::vector<cv::KeyPoint> kpts_from;
  cv::Mat image = frame_data->imageRaw();
  if (image.channels() > 1)
  {
    cv::Mat tmp;
    cv::cvtColor(image, tmp, cv::COLOR_BGR2GRAY);
    image = tmp;
  }

  cv::Mat depth_mask;
  if (!frame_data->depthRaw().empty())
  {
    if (image.rows % frame_data->depthRaw().rows == 0 &&
        image.cols % frame_data->depthRaw().cols == 0 &&
        image.rows / frame_data->depthRaw().rows ==
            frame_data->imageRaw().cols / frame_data->depthRaw().cols)
    {
      depth_mask = rtabmap::util2d::interpolate(
          frame_data->depthRaw(),
          frame_data->imageRaw().rows / frame_data->depthRaw().rows, 0.1f);
    }
    else
    {
      UWARN("%s is true, but RGB size (%dx%d) modulo depth size (%dx%d) is "
            "not 0. Ignoring depth mask for feature detection.",
            rtabmap::Parameters::kVisDepthAsMask().c_str(),
            frame_data->imageRaw().rows, frame_data->imageRaw().cols,
            frame_data->depthRaw().rows, frame_data->depthRaw().cols);
    }
  }

  rtabmap::ParametersMap registration_params;
  registration_params.insert(rtabmap::ParametersPair(
      rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
  auto detector = rtabmap::Feature2D::create(registration_params);

  auto kpts = detector->generateKeypoints(image, depth_mask);
  auto descriptors = detector->generateDescriptors(image, kpts);
  auto kpts3D = detector->generateKeypoints3D(*frame_data, kpts);

  frame_data->setFeatures(kpts, kpts3D, descriptors);

  // Clear costly data
  frame_data->clearCompressedData();
}

bool StereoHandler::generate_new_keyframe(std::shared_ptr<rtabmap::SensorData> &keyframe)
{
  // Keyframe generation heuristic
  bool generate_new_keyframe = true;
  if (generate_new_keyframes_based_on_inliers_ratio_)
  {
    if (nb_local_keyframes_ > 0)
    {
      rtabmap::RegistrationInfo reg_info;
      rtabmap::Transform t = registration_.computeTransformation(
          *keyframe, *previous_keyframe_, rtabmap::Transform(), &reg_info);
      if (!t.isNull())
      {
        if (float(reg_info.inliers) >
            keyframe_generation_ratio_ *
                float(previous_keyframe_->keypoints().size()))
        {
          generate_new_keyframe = false;
        }
      }
    }
    if (generate_new_keyframe)
    {
      previous_keyframe_ = keyframe;
    }
  }
  if (generate_new_keyframe)
  {
    // Store descriptors
    local_descriptors_map_.insert({keyframe->id(), keyframe});
    // Save frame to disk
    auto padded_image_id = std::to_string(keyframe->id());
    padded_image_id.insert(padded_image_id.begin(), 9 - padded_image_id.size(), '0');    
    auto image_path = results_save_path_ + "/samples/" + padded_image_id +
                 ".png";
    cv::imwrite(image_path, keyframe->imageRaw());
    keyframe->clearRawData();
    // Setup for next one
    nb_local_keyframes_++;
  }
  return generate_new_keyframe;
}

void StereoHandler::process_new_sensor_data()
{
  if (!received_data_queue_.empty())
  {
    auto sensor_data = received_data_queue_.front();
    received_data_queue_.pop_front();

    if (sensor_data.first->isValid())
    {
      cv::Mat rgb;
      sensor_data.first->uncompressDataConst(&rgb, 0);
      compute_local_descriptors(sensor_data.first);

      if (generate_new_keyframe(sensor_data.first))
      {
        send_keyframe(rgb, sensor_data.second, sensor_data.first->id());
      }
    }
  }
}

void StereoHandler::receive_local_keyframe_match(
    slam_interfaces::msg::LocalKeyframeMatches::ConstSharedPtr
        msg)
{
  auto keyframe0 = local_descriptors_map_.at(msg->keyframe0_id);
  keyframe0->uncompressData();
  auto keyframe1 = local_descriptors_map_.at(msg->keyframe1_id);
  keyframe1->uncompressData();
  rtabmap::RegistrationInfo reg_info;
  rtabmap::Transform t = registration_.computeTransformation(
      *keyframe0, *keyframe1, rtabmap::Transform(), &reg_info);

  slam_interfaces::msg::LoopClosure lc;
  lc.keyframe0_id = msg->keyframe0_id;
  lc.keyframe1_id = msg->keyframe1_id;
  if (!t.isNull())
  {
    lc.success = true;
    rtabmap_ros::transformToGeometryMsg(t, lc.transform);
  }
  else
  {
    lc.success = false;
  }
  loop_closure_publisher_->publish(lc);

  // Extract triplets of matches
  extract_triplets(msg->keyframe0_id, msg->keyframe1_id, reg_info, keyframe0, msg->other_matches_keyframe_ids);
}

void StereoHandler::extract_triplets(const unsigned int from_id, const unsigned int to_id, rtabmap::RegistrationInfo &reg_info, const std::shared_ptr<rtabmap::SensorData> tmp_from, const std::vector<unsigned int> &best_matches)
{
  // Find the closest match (VPR-wise) for which we cannot compute a transform
  auto it = best_matches.begin();
  std::vector<int> neg_ids;
  while (it != best_matches.end())
  {
    if (*it != to_id)
    {
      int match_id = *it;
      // Test this match if we can compute a transform
      // Compute transformation
      std::shared_ptr<rtabmap::SensorData> tmp_to = local_descriptors_map_.at(match_id);
      tmp_from->uncompressData();
      tmp_to->uncompressData();
      auto t = registration_.computeTransformation(*tmp_from, *tmp_to, rtabmap::Transform(), &reg_info);
      if (t.isNull())
      {
        neg_ids.push_back(match_id);
      }
    }
    // Try next one
    it++;
  }
  if (neg_ids.size() > 0)
  {
    std::ofstream triplets_file;
    triplets_file.open(results_save_path_ + "triplets.txt", std::ios::app);
    triplets_file << std::to_string(from_id) << " " << std::to_string(to_id);
    for (auto neg_id : neg_ids)
    {
      triplets_file << " " << std::to_string(neg_id);
    }
    triplets_file << "\n";
    triplets_file.close();
  }
}

void StereoHandler::send_keyframe(
    const rtabmap::SensorData &data,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom, const int id)
{
  // Image message
  std_msgs::msg::Header header;
  header.stamp = node_->now();
  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::RGB8, data.imageRaw());
  slam_interfaces::msg::KeyframeRGB keyframe_msg;
  image_bridge.toImageMsg(keyframe_msg.image);
  keyframe_msg.id = id;

  keyframe_data_publisher_->publish(keyframe_msg);

  slam_interfaces::msg::KeyframeOdom odom_msg;
  odom_msg.id = id;
  odom_msg.odom = *odom;
  keyframe_odom_publisher_->publish(odom_msg);
}