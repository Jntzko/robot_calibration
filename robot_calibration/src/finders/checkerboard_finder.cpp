/*
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

#include <pluginlib/class_list_macros.h>
#include <robot_calibration/capture/checkerboard_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(robot_calibration::CheckerboardFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

// We use a number of PC2 iterators, define the indexes here
const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

CheckerboardFinder::CheckerboardFinder() :
  waiting_(false)
{
}

bool CheckerboardFinder::init(const std::string& name,
                              ros::NodeHandle & nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  // Setup Scriber
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &CheckerboardFinder::cameraCallback,
                             this);

  // Size of checkerboard
  nh.param<int>("points_x", points_x_, 5);
  nh.param<int>("points_y", points_y_, 4);
  nh.param<double>("size", square_size_, 0.0245);

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

  // Publish where checkerboard points were seen
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  initMarker(nh);
  joint_states_sub_ = nh.subscribe("/joint_states", 1,&CheckerboardFinder::jointStatesCallback, this); 

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(nh))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

void CheckerboardFinder::jointStatesCallback(const sensor_msgs::JointState &msg)
{
  current_js_ = msg;
}

void CheckerboardFinder::cameraCallback(const sensor_msgs::PointCloud2& cloud)
{
  if (waiting_)
  {
    cloud_ = cloud;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout
bool CheckerboardFinder::waitForCloud()
{
  // Initial wait cycle so that camera is definitely up to date.
  ros::Duration(1/10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  ROS_ERROR("Failed to get cloud");
  return !waiting_;
}

bool CheckerboardFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  // Try up to 50 frames
  for (int i = 0; i < 50; ++i)
  {
    // temporary copy of msg, so we throw away all changes if findInternal() returns false
    robot_calibration_msgs::CalibrationData tmp_msg(*msg);
    if (findInternal(&tmp_msg))
    {
      *msg = tmp_msg;
      return true;
    }
  }
  return false;
}

bool CheckerboardFinder::findInternal(robot_calibration_msgs::CalibrationData * msg)
{
  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

  // Get cloud
  if (!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  // Get an image message from point cloud
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> rgb(cloud_, "rgb");
  image_msg->encoding = "bgr8";
  image_msg->height = cloud_.height;
  image_msg->width = cloud_.width;
  image_msg->step = image_msg->width * sizeof (uint8_t) * 3;
  image_msg->data.resize(image_msg->step * image_msg->height);
  for (size_t y = 0; y < cloud_.height; y++)
  {
    for (size_t x = 0; x < cloud_.width; x++)
    {
      uint8_t* pixel = &(image_msg->data[y * image_msg->step + x * 3]);
      pixel[0] = rgb[0];
      pixel[1] = rgb[1];
      pixel[2] = rgb[2];
      ++rgb;
    }
  }

  // Get an OpenCV image from the cloud
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(image_msg, "mono8");  // TODO: was rgb8? does this work?
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return false;
  }

  // Find checkerboard
  std::vector<cv::Point2f> points;
  points.resize(points_x_ * points_y_);
  cv::Size checkerboard_size(points_x_, points_y_);
  int found = cv::findChessboardCorners(bridge->image, checkerboard_size,
                                        points, CV_CALIB_CB_ADAPTIVE_THRESH);

  if (found)
  {
    ROS_INFO("Found the checkboard");

    // Create PointCloud2 to publish
    sensor_msgs::PointCloud2 cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = cloud_.header.frame_id;
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(points_x_ * points_y_);
    sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

    // Set msg size
    int idx_cam = msg->observations.size() + 0;
    int idx_chain = msg->observations.size() + 1;
    msg->observations.resize(msg->observations.size() + 2);
    msg->observations[idx_cam].sensor_name = camera_sensor_name_;
    msg->observations[idx_chain].sensor_name = chain_sensor_name_;
         
    msg->observations[idx_cam].features.resize(points_x_ * points_y_);
    msg->observations[idx_chain].features.resize(points_x_ * points_y_);


    // Fill in the headers
    rgbd.header = cloud_.header;
    world.header.frame_id = "checkerboard";

    // Fill in message
    sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud_, "x");
    for (size_t i = 0; i < points.size(); ++i)
    {
      world.point.x = (i % points_x_) * square_size_;
      world.point.y = (i / points_x_) * square_size_;

      // Get 3d point
      int index = (int)(points[i].y) * cloud_.width + (int)(points[i].x);
      rgbd.point.x = (xyz + index)[X];
      rgbd.point.y = (xyz + index)[Y];
      rgbd.point.z = (xyz + index)[Z];

      // Do not accept NANs
      if (isnan(rgbd.point.x) ||
          isnan(rgbd.point.y) ||
          isnan(rgbd.point.z))
      {
        ROS_ERROR_STREAM("NAN point on " << i);
        return false;
      }

      msg->observations[idx_cam].features[i] = rgbd;
      msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
      msg->observations[idx_chain].features[i] = world;

      // Visualize
      iter_cloud[0] = rgbd.point.x;
      iter_cloud[1] = rgbd.point.y;
      iter_cloud[2] = rgbd.point.z;
      ++iter_cloud;
    }

    // Add debug cloud to message
    if (output_debug_)
    {
      msg->observations[idx_cam].cloud = cloud_;
    }

    msg->joint_states = current_js_;

    visualizeMarker(*msg);

    // Publish results
    publisher_.publish(cloud);

    // Found all points
    return true;
  }

  return false;
}

void CheckerboardFinder::initMarker(ros::NodeHandle &nh) {
  ros::NodeHandle pnh("~");
  params_.LoadFromROS(pnh);
  ROS_WARN_STREAM("Publishing markers in " << params_.base_link << " frame.");

  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("marker",10);
  // Index 0 is white -- used for first points
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.b = 1.0;
  color.g = 1.0;
  color.a = 1.0;
  model_colors_.push_back(color);
  // Red
  color.r = 1;
  color.g = 0;
  color.b = 0;
  color.a = 1;
  model_colors_.push_back(color);
  // Green
  color.r = 0;
  color.g = 1;
  color.b = 0;
  color.a = 1;
  model_colors_.push_back(color);
  // Green
  color.r = 0;
  color.g = 0;
  color.b = 1;
  color.a = 1;
  model_colors_.push_back(color);

  std::string str;
  if (!nh.param<std::string>("/robot_description", str, "asd"))
  {
    ROS_ERROR("No robot description");
    return;
  }

  // Load KDL from URDF
  urdf::Model model;
  KDL::Tree tree;
  if (!model.initString(str))
  {
    ROS_FATAL("Failed to parse URDF.");
  }
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_FATAL("Failed to construct KDL tree");
  }

  // Create models for reprojection
//  std::vector<std::string> model_names;
  for (size_t i = 0; i < params_.models.size(); ++i)
  {
    if (params_.models[i].type == "chain")
    {
      robot_calibration::ChainModel* model = new robot_calibration::ChainModel(params_.models[i].name, tree, params_.base_link, params_.models[i].params["frame"]);
      models_[params_.models[i].name] = model;
//      model_names.push_back(params_.models[i].name);
    }
    else
    {
      // ERROR unknown
    }
  }

  // Load initial values for offsets (if any)
  for (size_t i = 0; i < params_.free_params.size(); ++i)
  {
    offsets_.add(params_.free_params[i]);
  }
  for (size_t i = 0; i < params_.free_frames.size(); ++i)
  {
    offsets_.addFrame(params_.free_frames[i].name,
                     params_.free_frames[i].x,
                     params_.free_frames[i].y,
                     params_.free_frames[i].z,
                     params_.free_frames[i].roll,
                     params_.free_frames[i].pitch,
                     params_.free_frames[i].yaw);
  }
  for (size_t i = 0; i < params_.free_frames_initial_values.size(); ++i)
  {
    if (!offsets_.setFrame(params_.free_frames_initial_values[i].name,
                          params_.free_frames_initial_values[i].x,
                          params_.free_frames_initial_values[i].y,
                          params_.free_frames_initial_values[i].z,
                          params_.free_frames_initial_values[i].roll,
                          params_.free_frames_initial_values[i].pitch,
                          params_.free_frames_initial_values[i].yaw))
    {
      ROS_ERROR_STREAM("Error setting initial value for " <<
                       params_.free_frames_initial_values[i].name);
    }
    ROS_ERROR_STREAM(params_.free_frames_initial_values[i].x << "  " << params_.free_frames_initial_values[i].y << "   "  << params_.free_frames_initial_values[i].z);
  }
}

void CheckerboardFinder::visualizeMarker(robot_calibration_msgs::CalibrationData msg) {

  // Publish a marker array
  visualization_msgs::MarkerArray marker_array;
  for(size_t i = 0; i < msg.observations.size(); ++i)
  {
    visualization_msgs::Marker marker;
    for (size_t j = 0; j < params_.models.size(); ++j)
    {
      if (msg.observations[i].sensor_name == params_.models[j].name)
        marker.header.frame_id = static_cast<std::string>(params_.models[j].params["frame"]);
    }
    marker.header.stamp = ros::Time::now();
    marker.ns = msg.observations[i].sensor_name;
    marker.id = i;
    marker.type = marker.SPHERE_LIST;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.points.push_back(msg.observations[i].features[0].point);
    marker.colors.push_back(model_colors_[0]);
    for (size_t p = 1; p < msg.observations[i].features.size(); ++p)
    {
      marker.points.push_back(msg.observations[i].features[p].point);
      marker.colors.push_back(model_colors_[i+1]);
    }
    if (params_.free_frames_initial_values.size() > 0 && msg.observations[i].sensor_name == "left_arm") {
      marker.pose.position.x = params_.free_frames_initial_values[0].x;
      marker.pose.position.y = params_.free_frames_initial_values[0].y;
      marker.pose.position.z = params_.free_frames_initial_values[0].z;
      marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(params_.free_frames_initial_values[0].roll,params_.free_frames_initial_values[0].pitch, params_.free_frames_initial_values[0].yaw);
    }
    marker_array.markers.push_back(marker);
  }

  // Visualization as in the viz.cpp
  // Project through model
  std::vector<geometry_msgs::PointStamped> points;
  points = models_["left_arm"]->project(msg, offsets_);

  // Convert into marker
  visualization_msgs::Marker ms;
  ms.header.frame_id = params_.base_link;
  ms.header.stamp = ros::Time::now();
  ms.ns = "left_arm_2";
  ms.id = 2;
  ms.type = ms.SPHERE_LIST;
  ms.scale.x = 0.005;
  ms.scale.y = 0.005;
  ms.scale.z = 0.005;
  ms.points.push_back(points[0].point);
  ms.colors.push_back(model_colors_[0]);
  for (size_t p = 1; p < points.size(); ++p)
  {
    ms.points.push_back(points[p].point);
    ms.colors.push_back(model_colors_[3]);
  }
  marker_array.markers.push_back(ms);


  marker_publisher_.publish(marker_array);
}

}  // namespace robot_calibration
