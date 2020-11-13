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

// Author: Yannick Jonetzko

#include <robot_calibration/capture/apriltag_finder.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf/tf.h>


PLUGINLIB_EXPORT_CLASS(robot_calibration::AprilTagFinder, robot_calibration::FeatureFinder)

namespace robot_calibration
{

AprilTagFinder::AprilTagFinder() :
  waiting_(false)
{
}

bool AprilTagFinder::init(const std::string& name, ros::NodeHandle& nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  tag_detector_ = new apriltag_ros::TagDetector(nh);
  // Setup Subscriber
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/camera/image_raw");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &AprilTagFinder::cameraCallback,
                             this);

  // Get Standalone tag ids and sizes
  XmlRpc::XmlRpcValue standalone_tag_descriptions;
  if(nh.getParam("standalone_tags", standalone_tag_descriptions)) {
    tags_ = tag_detector_->parseStandaloneTags(standalone_tag_descriptions);
  }

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

  // Publish where apriltag points were seen
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  // Setup to get camera info
  if (!depth_camera_manager_.init(nh))
  {
    // Error will have been printed by manager
    return false;
  }

  return true;
}

void AprilTagFinder::cameraCallback(const sensor_msgs::Image::ConstPtr& image)
{
  if (waiting_)
  {
    image_ = *image;
    waiting_ = false;
  }

}

// Returns true if we got a message, false if we timeout
bool AprilTagFinder::waitForImage()
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
  ROS_ERROR("Failed to get image");
  return !waiting_;
}


bool AprilTagFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

 //  Get image
 if (!waitForImage())
  {
    ROS_ERROR("No image data");
    return false;
  }


  cv_bridge::CvImagePtr cv_image_ptr;
  try
  {
    cv_image_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  boost::shared_ptr<sensor_msgs::CameraInfo> camera_info_ptr(new sensor_msgs::CameraInfo);
  *camera_info_ptr = depth_camera_manager_.getDepthCameraInfo().camera_info;
  apriltag_ros::AprilTagDetectionArray tag_detections =  tag_detector_->detectTags(cv_image_ptr, camera_info_ptr);

  if (tag_detections.detections.size() > 0)
  {
    apriltag_ros::AprilTagDetection tag;
    for (size_t i=0; i<tag_detections.detections.size(); i++)
    {
      tag = tag_detections.detections[i];
      if (tags_.find(tag.id[0]) != tags_.end())
      {
        ROS_INFO_STREAM("Found AprilTag with id: " << tag.id[0]);

        // We have just a point in observation, so use corners and origin of AprilTag
        double size = tags_.find(tag.id[0])->second.size();

        std::vector<tf::Vector3> points;
        for (size_t j=0; j<5; j++)
        {
          for (size_t k=0; k<4; k++)
          {
            float x = j*size/4;
            float y = k*size/3;
            points.push_back(tf::Vector3(x-size/2, y-size/2, 0));
          }
        }

        // Create PointCloud2 to publish
        sensor_msgs::PointCloud2 cloud;
        cloud.width = 0;
        cloud.height = 0;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = image_.header.frame_id;
        sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
        cloud_mod.setPointCloud2FieldsByString(1, "xyz");
        cloud_mod.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

        // Set msg size
        int idx_cam = msg->observations.size() + 0;
        int idx_chain = msg->observations.size() + 1;
        msg->observations.resize(msg->observations.size() + 2);
        msg->observations[idx_cam].sensor_name = camera_sensor_name_;
        msg->observations[idx_chain].sensor_name = chain_sensor_name_;

        msg->observations[idx_cam].features.resize(points.size());
        msg->observations[idx_chain].features.resize(points.size());

        // Fill in the headers
        rgbd.header = image_.header;

        // Fill in message
        for (size_t i = 0; i < points.size(); ++i)
        {
          std::stringstream out;
          out << "tag_" << tag.id[0];
          world.header.frame_id = out.str();

          // Point in relation to the chain
          // Origin
          world.point.x = points[i].getX();
          world.point.y = points[i].getY();

          // Point in relation to the camera
          // Get 3d point
          tf::Quaternion q;
          tf::quaternionMsgToTF(tag.pose.pose.pose.orientation, q);
          tf::Vector3 point = tf::quatRotate(q, points[i]);

          rgbd.point.x = point.getX() + tag.pose.pose.pose.position.x;
          rgbd.point.y = point.getY() + tag.pose.pose.pose.position.y;
          rgbd.point.z = point.getZ() + tag.pose.pose.pose.position.z;

          msg->observations[idx_cam].features[i] = rgbd;
          msg->observations[idx_cam].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
          msg->observations[idx_chain].features[i] = world;

          // Visualize
          iter_cloud[0] = rgbd.point.x;
          iter_cloud[1] = rgbd.point.y;
          iter_cloud[2] = rgbd.point.z;
          ++iter_cloud;
        }

        // Publish results
        publisher_.publish(cloud);
      }
    }
    // Found all points
    return true;
  }

  return false;
}

}  // namespace robot_calibration
