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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

#include "swm_ros/swm_ros.h"

DEFINE_string(config_filename, "", "Path and filename of swm_zyre_config.json");
DEFINE_string(topic_raw_image, "/cam0/image_raw", "Raw image topic.");
DEFINE_string(topic_estimated_position, "/mavros/global_position/global",
              "Estimated position of the robot");
DEFINE_string(topic_estimated_orientation, "/mavros/imu/data",
              "Estimated orientation of the robot");
DEFINE_string(topic_victim_position, "/victim_position",
              "Position of the victim in lat/lon/alt");
DEFINE_string(agent_name, "fw0", "Name of the agent.");
DEFINE_string(image_base_name, "/tmp/image", "Image directory and base name.");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "swm_zyre");
  ros::Time::init();

  sherpa::SwmRos app(FLAGS_config_filename,
                     FLAGS_topic_raw_image,
                     FLAGS_topic_estimated_position,
                     FLAGS_topic_estimated_orientation,
                     FLAGS_topic_victim_position,
                     FLAGS_agent_name, FLAGS_image_base_name);
  app.processLiveRosStream();

  ros::spin();

  return 0;
}
