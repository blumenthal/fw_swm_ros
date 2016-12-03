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

  SwmRos(sherpa::mode mode, std::string config_filename,
         std::string rosbag_filename, std::string topic_raw_image,
         std::string topic_estimated_robot_position,
         std::string topic_estimated_robot_orientation,
         std::string topic_victim_position, std::string agent_name,
         std::string image_base_name);

  /// Real-data, loaded from rosbag.
  void processRosbag();

  /// Real-data, extracted from ros message stream.
  void processLiveRosStream();

 private:
  void estimatedRobotPositionCallback(const sensor_msgs::NavSatFixConstPtr& message);
  void estimatedRobotOrientationCallback(const sensor_msgs::ImuConstPtr& message);
  void victimPositionCallback(const sensor_msgs::NavSatFixConstPtr& message);
  void rawImageCallback(const sensor_msgs::ImageConstPtr& message);

  void registerSubscriber();

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

  /// Counter for the images saved to the ground station.
  int64_t image_counter_;
};
} // namespace sherpa
#endif // SWM_ROS_H_
