/*
 * Copyright (C) 2018 Michael Ferguson
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Yannick Jonetzko

#ifndef ROBOT_CALIBRATION_CAPTURE_APRILTAGS2_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_APRILTAGS2_FINDER_H

#include <ros/ros.h>
#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/plugins/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <apriltag_ros/common_functions.h>
#include <sensor_msgs/Image.h>

#include "apriltag.h"

namespace robot_calibration
{

/**
 *  \brief This class processes the camera input to find an apriltag
 */
class AprilTagFinder : public FeatureFinder
{
public:
  AprilTagFinder();
  bool init(const std::string& name, ros::NodeHandle & n);
  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  void cameraCallback(const sensor_msgs::Image::ConstPtr& image);
  bool waitForImage();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::Image
  ros::Publisher publisher_;   /// Outgoing sensor_msgs::PointCloud2

  bool waiting_;
  sensor_msgs::Image image_;
  DepthCameraInfoManager depth_camera_manager_;

  apriltag_ros::TagDetector* tag_detector_;

  /*
   * ROS Parameters
   */
  std::map<int, apriltag_ros::StandaloneTagDescription> tags_;

  bool output_debug_;   /// Should we output debug image/cloud?

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_APRILTAGS_FINDER_H
