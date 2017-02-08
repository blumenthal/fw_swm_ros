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

#include "swm_ros/swm_ros.h"
#include "swm_ros/gps_conversions.h"

namespace sherpa {

// Ctor for node_rosbag.
SwmRos::SwmRos(std::string config_filename,
               std::string rosbag_filename, std::string topic_raw_image,
               std::string topic_estimated_robot_position,
               std::string topic_estimated_robot_orientation,
               std::string topic_victim_position, std::string agent_name,
               std::string image_base_name) :
  config_filename_(config_filename), rosbag_filename_(rosbag_filename),
  topic_raw_image_(topic_raw_image),
  topic_estimated_robot_position_(topic_estimated_robot_position),
  topic_estimated_robot_orientation_(topic_estimated_robot_orientation),
  topic_victim_position_(topic_victim_position), agent_name_(&agent_name[0u]),
  image_base_name_(image_base_name), image_transport_(node_handle_) {
  CHECK(rosbag_filename_ != "") << "You need to enter path to rosbag.";
  initialize();
}

// Ctor for node_rosstream.
SwmRos::SwmRos(std::string config_filename,
               std::string topic_raw_image,
               std::string topic_estimated_robot_position,
               std::string topic_estimated_robot_orientation,
               std::string topic_victim_position, std::string agent_name,
               std::string image_base_name) :
  config_filename_(config_filename), topic_raw_image_(topic_raw_image),
  topic_estimated_robot_position_(topic_estimated_robot_position),
  topic_estimated_robot_orientation_(topic_estimated_robot_orientation),
  topic_victim_position_(topic_victim_position), agent_name_(&agent_name[0u]),
  image_base_name_(image_base_name), image_transport_(node_handle_) {
  initialize();
  registerSubscriber();
  registerServices();
}

void SwmRos::initialize() {
  // Load configuration file for communication setup.
  VLOG(1) << "Config filename: '" << config_filename_ << "'";
  json_t* config = load_config_file(const_cast<char*>(config_filename_.c_str()));
  CHECK(config == NULL) << "Config not loaded successfully.";

  // Spawn new communication component.
  self_ = new_component(config);
  CHECK(self_ == NULL) << "Config not loaded successfully.";

  printf("[%s] component initialized!\n", self_->name);
  free(config);

  agent_added_ = false;
  new_robot_position_estimate_available_ = false;
  new_robot_orientation_estimate_available_ = false;
  robot_pose_initialized_ = false;
  current_utc_time_ms_ = 0.0;
  image_counter_ = 0;
}

bool SwmRos::insertImage(swm_ros::insertImageIntoSwm::Request &req,
                         swm_ros::insertImageIntoSwm::Response &resp) {
  double utc_time_ms = rosToMilliSeconds(ros::Time::now());
  assert(add_image(self_, current_robot_pose_.data(), utc_time_ms, agent_name_,
                   &req.path_to_image[0u]));
  resp.success.data = true;
}

bool SwmRos::insertHumanDetection(swm_ros::insertHumanDetectionIntoSwmUTM::Request &req,
                                  swm_ros::insertHumanDetectionIntoSwmUTM::Response &resp) {
  double utc_time_ms = req.time_of_observation_ns * 1.0e-6;
  // To avoid misunderstandings:
  const double northing = req.human_location_UTM.y;
  const double easting = req.human_location_UTM.x;
  const double altitude = req.human_location_UTM.z;

  // Switzerland.
  double latitude, longitude;
  UTM::UTMtoLL(northing, easting, UTM_zone_, latitude, longitude);
  VLOG(1) << "Human detection at: [lat/long/alt]: [" << latitude << "/"
          << longitude << "/" << altitude << "] <==> [easting/northing/alt]: ["
          << easting << "/" << northing << "/" << altitude << "]";

  Eigen::Matrix4d victim_pose;
  victim_pose.setIdentity();
  victim_pose.block<3, 1>(0, 3) << Eigen::Vector3d(latitude, longitude, altitude);
  assert(add_victim(self_, victim_pose.data(), utc_time_ms, agent_name_));
  resp.success.data = true;
}

void SwmRos::registerServices() {
  service_image_insertion_ =
      node_handle_.advertiseService("/fw_swm_ros/image_insertion", &SwmRos::insertImage, this);
  service_human_detection_insertion_ =
      node_handle_.advertiseService("/fw_swm_ros/human_detection_insertion",
                                    &SwmRos::insertHumanDetection, this);
}

void SwmRos::registerSubscriber() {
  // Raw image subscriber.
  boost::function<void(const sensor_msgs::ImageConstPtr& msg)> raw_image_callback = boost::bind(
        &SwmRos::rawImageCallback, this, _1);
  image_transport::Subscriber sub_raw_image_ =
      image_transport_.subscribe(topic_raw_image_, 20, raw_image_callback);

  // Robot orientation subscriber.
  LOG(INFO) << "Subscribed to 'robot orientation topic': " << topic_estimated_robot_orientation_;
  boost::function<void(const sensor_msgs::ImuConstPtr& msg)> estimated_robot_orientation_callback
      = boost::bind(&SwmRos::estimatedRobotOrientationCallback, this, _1);
  sub_estimated_robot_orientation_ =
      node_handle_.subscribe(topic_estimated_robot_orientation_, 1000, estimated_robot_orientation_callback);

  // Robot position subscriber.
  LOG(INFO) << "Subscribed to 'robot position topic': " << topic_estimated_robot_position_;
  boost::function<void(const sensor_msgs::NavSatFixConstPtr& msg)> estimated_robot_position_callback
      = boost::bind(&SwmRos::estimatedRobotPositionCallback, this, _1);
  sub_estimated_robot_position_ =
      node_handle_.subscribe(topic_estimated_robot_position_, 1000, estimated_robot_position_callback);

  // Victim position subscriber.
  LOG(INFO) << "Subscribed to 'victim position topic': " << topic_victim_position_;
  boost::function<void(const sensor_msgs::NavSatFixConstPtr& msg)> victim_position_callback
      = boost::bind(&SwmRos::victimPositionCallback, this, _1);
  sub_victim_position_ =
      node_handle_.subscribe(topic_victim_position_, 1000, victim_position_callback);
}

void SwmRos::rawImageCallback(const sensor_msgs::ImageConstPtr& image_message) {
  if (robot_pose_initialized_) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      // Convert the image to MONO8 if necessary.
      cv_ptr = cv_bridge::toCvShare(image_message, sensor_msgs::image_encodings::MONO8);
    } catch(const cv_bridge::Exception& e) {
      LOG(FATAL) << "cv_bridge exception: " << e.what();
    }
    CHECK(cv_ptr);
    // Save image locally.
    std::string image_name = image_base_name_ + std::to_string(++image_counter_) + ".jpg";
    cv::imwrite(image_name, cv_ptr->image.clone());
    double utc_time_ms = image_message->header.stamp.toSec() * 1.0e3;
    assert(add_image(self_, current_robot_pose_.data(), utc_time_ms, agent_name_,
                     &image_name[0u]));
  }
}

void SwmRos::estimatedRobotPositionCallback(const sensor_msgs::NavSatFixConstPtr& position_message) {
  current_robot_position_ = Eigen::Vector3d(position_message->latitude,
                                            position_message->longitude,
                                            position_message->altitude);
  current_utc_time_ms_ = position_message->header.stamp.toSec() * 1.0e3;
  new_robot_position_estimate_available_ = true;
}

void SwmRos::estimatedRobotOrientationCallback(const sensor_msgs::ImuConstPtr& orientation_message) {
  current_robot_orientation_ = Eigen::Quaterniond(orientation_message->orientation.w,
                                                  orientation_message->orientation.x,
                                                  orientation_message->orientation.y,
                                                  orientation_message->orientation.z);
  current_utc_time_ms_ = orientation_message->header.stamp.toSec() * 1.0e3;
  new_robot_orientation_estimate_available_ = true;
}

void SwmRos::victimPositionCallback(const sensor_msgs::NavSatFixConstPtr& victim_position_message) {
  Eigen::Vector3d victim_position (victim_position_message->latitude,
                                   victim_position_message->longitude,
                                   victim_position_message->altitude);
  Eigen::Matrix4d victim_pose;
  victim_pose.setIdentity();
  victim_pose.block<3, 1>(0, 3) << victim_position;
  double utc_time_ms = victim_position_message->header.stamp.toSec() * 1.0e3;
  assert(add_victim(self_, victim_pose.data(), utc_time_ms, agent_name_));
}

void SwmRos::processLiveRosStream() {
  // Probably smart to limit the rate?
  ros::Rate(1.0);
  // Add agent (if not added yet) and update robot pose.
  if (new_robot_position_estimate_available_ &&
      new_robot_orientation_estimate_available_) {
    current_robot_pose_.block<3, 3>(0, 0) << current_robot_orientation_.toRotationMatrix();
    current_robot_pose_.block<3, 1>(3, 0) << current_robot_position_;

    if (!agent_added_) {
      assert(add_agent(self_, current_robot_pose_.data(), current_utc_time_ms_, agent_name_));
      agent_added_ = true;
    }

    assert(update_pose(self_, current_robot_pose_.data(), current_utc_time_ms_, agent_name_));
    new_robot_orientation_estimate_available_ = false;
    new_robot_position_estimate_available_ = false;
    robot_pose_initialized_ = true;
  }
}

void SwmRos::processRosbag() {
  try {
    bag_.reset(new rosbag::Bag);
    bag_->open(rosbag_filename_, rosbag::bagmode::Read);
  } catch(const std::exception& ex) {
    LOG(FATAL) << "Could not open the rosbag " << rosbag_filename_
               << ": " << ex.what();
  }
  std::vector<std::string> all_topics;
  all_topics.push_back(topic_raw_image_);
  all_topics.push_back(topic_estimated_robot_position_);
  all_topics.push_back(topic_estimated_robot_orientation_);
  all_topics.push_back(topic_victim_position_);

  const double rosbag_end_s = 0.0;
  const double rosbag_start_s = 0.0;
  try {
    CHECK(bag_);
    if (rosbag_end_s != 0.0 || rosbag_start_s != 0.0) {
      CHECK_GE(rosbag_start_s, 0);
      CHECK_GT(rosbag_end_s, rosbag_start_s);
      // Get the start offset from the unconstrained view.
      bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(all_topics)));
      double absolute_time_offset = bag_view_->getBeginTime().toSec();
      double absolute_start_time_s = absolute_time_offset + rosbag_start_s;
      double absolute_end_time_s = absolute_time_offset + rosbag_end_s;
      bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(all_topics),
                                       ros::Time(absolute_start_time_s),
                                       ros::Time(absolute_end_time_s)));
    } else {
      bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(all_topics)));
    }
  } catch(const std::exception& ex) {
    LOG(FATAL) << "Could not open a rosbag view: " << ex.what();
  }
  // Play all messages.
  CHECK(bag_view_);

  for (const rosbag::MessageInstance& message : *bag_view_) {
    // Add image messages to SHERPA World Model (SVM).
    sensor_msgs::ImageConstPtr image_message = message.instantiate<sensor_msgs::Image>();
    if (image_message && robot_pose_initialized_) {
      CHECK(image_message);
      // Get the image.
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        // Convert the image to MONO8 if necessary.
        cv_ptr = cv_bridge::toCvShare(image_message, sensor_msgs::image_encodings::MONO8);
      } catch(const cv_bridge::Exception& e) {
        LOG(FATAL) << "cv_bridge exception: " << e.what();
      }
      CHECK(cv_ptr);

      // Save image locally.
      std::string image_name = image_base_name_ + std::to_string(++image_counter_) + ".jpg";
      cv::imwrite(image_name, cv_ptr->image.clone());
      double utc_time_ms = image_message->header.stamp.toSec() * 1.0e3;
      assert(add_image(self_, current_robot_pose_.data(), utc_time_ms, agent_name_, &image_name[0u]));
    }

    // Add compressed image messages to SHERPA World Model (SVM).
    sensor_msgs::CompressedImageConstPtr compressed_image_message =
        message.instantiate<sensor_msgs::CompressedImage>();
    if (compressed_image_message && robot_pose_initialized_) {
      CHECK(compressed_image_message);
      // Get the image.
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        // Convert the image to MONO8 if necessary.
        cv_ptr = cv_bridge::toCvCopy(compressed_image_message, sensor_msgs::image_encodings::MONO8);
      } catch(const cv_bridge::Exception& e) {
        LOG(FATAL) << "cv_bridge exception: " << e.what();
      }
      CHECK(cv_ptr);

      // Save image locally.
      std::string image_name = image_base_name_ + std::to_string(++image_counter_) + ".jpg";
      cv::imwrite(image_name, cv_ptr->image.clone());
      double utc_time_ms = compressed_image_message->header.stamp.toSec() * 1.0e3;
      assert(add_image(self_, current_robot_pose_.data(), utc_time_ms, agent_name_, &image_name[0u]));
    }

    // Update robot position estimate.
    if (message.getTopic() == topic_estimated_robot_position_) {
      sensor_msgs::NavSatFixConstPtr position_message =
          message.instantiate<sensor_msgs::NavSatFix>();
      current_robot_position_ = Eigen::Vector3d(position_message->latitude,
                                                position_message->longitude,
                                                position_message->altitude);
      current_utc_time_ms_ = position_message->header.stamp.toSec() * 1.0e3;
      new_robot_position_estimate_available_ = true;
    }

    // Update robot orientation estimate.
    if (message.getTopic() == topic_estimated_robot_orientation_) {
      sensor_msgs::Imu::ConstPtr orientation_message = message.instantiate<sensor_msgs::Imu>();
      current_robot_orientation_ = Eigen::Quaterniond(orientation_message->orientation.w,
                                                      orientation_message->orientation.x,
                                                      orientation_message->orientation.y,
                                                      orientation_message->orientation.z);
      current_utc_time_ms_ = orientation_message->header.stamp.toSec() * 1.0e3;
      new_robot_orientation_estimate_available_ = true;
    }

    // Add victim detection.
    if (message.getTopic() == topic_victim_position_ && robot_pose_initialized_) {
      sensor_msgs::NavSatFixConstPtr victim_position_message =
          message.instantiate<sensor_msgs::NavSatFix>();
      Eigen::Vector3d victim_position (victim_position_message->latitude,
                                       victim_position_message->longitude,
                                       victim_position_message->altitude);
      Eigen::Matrix4d victim_pose;
      victim_pose.setIdentity();
      victim_pose.block<3, 1>(0, 3) << victim_position;
      double utc_time_ms = victim_position_message->header.stamp.toSec() * 1.0e3;
      assert(add_victim(self_, victim_pose.data(), utc_time_ms, agent_name_));
    }

    // Add agent (if not added yet) and update robot pose.
    if (new_robot_position_estimate_available_ &&
        new_robot_orientation_estimate_available_) {
      current_robot_pose_.block<3, 3>(0, 0) << current_robot_orientation_.toRotationMatrix();
      current_robot_pose_.block<3, 1>(0, 3) << current_robot_position_;

      if (!agent_added_) {
        assert(add_agent(self_, current_robot_pose_.data(), current_utc_time_ms_,
                         agent_name_));
        agent_added_ = true;
      }

      assert(update_pose(self_, current_robot_pose_.data(), current_utc_time_ms_,
                         agent_name_));
      new_robot_orientation_estimate_available_ = false;
      new_robot_position_estimate_available_ = false;
      robot_pose_initialized_ = true;
    }
  }
}
} // namespace sherpa
