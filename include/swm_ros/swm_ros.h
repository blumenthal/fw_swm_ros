// This file is part of fw_swm_ros.
//
// Copyright (C) 2016 Timo Hinzmann <hitimo at ethz dot ch> 
// (Autonomous Systems Lab, ETH Zurich, Switzerland)
//
// fw_swm_ros is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// fw_swm_ros is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with fw_swm_ros.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SWM_ROS_H_
#define SWM_ROS_H_

#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <swm_ros/insertImageIntoSwm.h>
#include <swm_ros/insertHumanDetectionIntoSwmUTM.h>


extern "C" {
#include "/usr/local/include/swmzyre.h"
}

namespace sherpa {
enum mode {
  rosbag,
  rosstream
};

class SwmRos {
 public:

  SwmRos(std::string config_filename,
         std::string rosbag_filename, std::string topic_raw_image,
         std::string topic_estimated_robot_position,
         std::string topic_estimated_robot_orientation,
         std::string topic_victim_position, std::string agent_name,
         std::string image_base_name);

  SwmRos(std::string config_filename,
         std::string topic_raw_image,
         std::string topic_estimated_robot_position,
         std::string topic_estimated_robot_orientation,
         std::string topic_victim_position, std::string agent_name,
         std::string image_base_name);

  /// Real-data, loaded from rosbag.
  void processRosbag();

  /// Real-data, extracted from ros message stream.
  void processLiveRosStream();

 private:
  void initialize();
  void estimatedRobotPositionCallback(const sensor_msgs::NavSatFixConstPtr& message);
  void estimatedRobotOrientationCallback(const sensor_msgs::ImuConstPtr& message);
  void victimPositionCallback(const sensor_msgs::NavSatFixConstPtr& message);
  void rawImageCallback(const sensor_msgs::ImageConstPtr& message);

  void registerSubscriber();

  void registerServices();

  bool insertImage(swm_ros::insertImageIntoSwm::Request &req,
                   swm_ros::insertImageIntoSwm::Response &resp);
  bool insertHumanDetection(swm_ros::insertHumanDetectionIntoSwmUTM::Request &req,
                            swm_ros::insertHumanDetectionIntoSwmUTM::Response &resp);

  double inline rosToSeconds(const ros::Time time) {
   return time.sec + time.nsec / 1.0e9;
 }

  double inline rosToMilliSeconds(const ros::Time time) {
   return (time.sec + time.nsec / 1.0e9) * 1.0e3;
 }

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> bag_view_;
  component_t* self_;

  std::string rosbag_filename_;
  std::string topic_raw_image_;
  std::string topic_estimated_robot_position_;
  std::string topic_estimated_robot_orientation_;
  std::string topic_victim_position_;
  char* agent_name_;
  std::string image_base_name_;

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  ros::Subscriber sub_estimated_robot_orientation_;
  ros::Subscriber sub_estimated_robot_position_;
  ros::Subscriber sub_victim_position_;

  bool agent_added_;
  bool new_robot_position_estimate_available_;
  bool new_robot_orientation_estimate_available_;
  bool robot_pose_initialized_;
  Eigen::Matrix4d current_robot_pose_;
  Eigen::Vector3d current_robot_position_;
  Eigen::Quaterniond current_robot_orientation_;
  double current_utc_time_ms_;
  std::string config_filename_;
  // E.g. Switzerland.
  char const *UTM_zone_ = "32T";

  /// Counter for the images saved to the ground station.
  int64_t image_counter_;
  ros::ServiceServer service_image_insertion_;
  ros::ServiceServer service_human_detection_insertion_;

};
} // namespace sherpa
#endif // SWM_ROS_H_
