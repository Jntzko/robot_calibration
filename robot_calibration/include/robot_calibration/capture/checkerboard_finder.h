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

// Author: Michael Ferguson

#ifndef ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H

#include <ros/ros.h>
#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/plugins/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/MarkerArray.h>
#include <robot_calibration/ceres/optimization_params.h>

#include <robot_calibration/models/chain.h>
#include <robot_calibration/calibration_offset_parser.h>

namespace robot_calibration
{

/**
 *  \brief This class processes the point cloud input to find a checkerboard
 */
class CheckerboardFinder : public FeatureFinder
{
public:
  CheckerboardFinder();
  bool init(const std::string& name, ros::NodeHandle & n);
  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  bool findInternal(robot_calibration_msgs::CalibrationData * msg);

  void cameraCallback(const sensor_msgs::PointCloud2& cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2
  ros::Publisher publisher_;   /// Outgoing sensor_msgs::PointCloud2

  bool waiting_;
  sensor_msgs::PointCloud2 cloud_;
  DepthCameraInfoManager depth_camera_manager_;

  /*
   * ROS Parameters
   */
  int points_x_;        /// Size of checkerboard
  int points_y_;        /// Size of checkerboard

  double square_size_;     /// Size of a square on checkboard (in meters)

  bool output_debug_;   /// Should we output debug image/cloud?

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;

  void visualizeMarker(robot_calibration_msgs::CalibrationData msg);
  void initMarker(ros::NodeHandle &nh);
  robot_calibration::OptimizationParams params_;
  ros::Publisher marker_publisher_;
  std::vector<std_msgs::ColorRGBA> model_colors_;
  robot_calibration::CalibrationOffsetParser offsets_;
  std::map<std::string, robot_calibration::ChainModel*> models_;
  ros::Subscriber joint_states_sub_;
  void jointStatesCallback(const sensor_msgs::JointState & msg);
  sensor_msgs::JointState current_js_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
