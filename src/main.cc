#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

#include "swm_ros/swm_ros.h"

DEFINE_string(config_filename, "", "Path and filename of swm_zyre_config.json");
DEFINE_string(rosbag_filename, "", "Path and filename of rosbag");
DEFINE_string(topic_raw_image, "/cam0/image_raw", "Raw image topic.");
DEFINE_string(topic_estimated_position, "/mavros/global_position/global",
              "Estimated position of the robot");
DEFINE_string(topic_estimated_orientation, "/mavros/imu/data",
              "Estimated orientation of the robot");
DEFINE_string(topic_victim_position, "/victim_position",
              "Position of the victim in lat/lon/alt");
DEFINE_string(agent_name, "fw0", "Name of the agent.");
DEFINE_string(image_base_name, "/tmp/image", "Image directory and base name.");
DEFINE_int32(mode, 0, "mode 0: Load Rosbag; mode 1: Live Ros stream");


int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(FLAGS_config_filename != "") << "You need to enter path to config.";
  ros::init(argc, argv, "swm_zyre");
  ros::Time::init();

  sherpa::mode selected_mode = static_cast<sherpa::mode>(FLAGS_mode);
  sherpa::SwmRos app(selected_mode, FLAGS_config_filename,
                     FLAGS_rosbag_filename, FLAGS_topic_raw_image,
                     FLAGS_topic_estimated_position,
                     FLAGS_topic_estimated_orientation,
                     FLAGS_topic_victim_position,
                     FLAGS_agent_name, FLAGS_image_base_name);

  switch (selected_mode) {
  case sherpa::mode::rosbag:
    app.processRosbag();
    break;
  case sherpa::mode::rosstream:
    app.processLiveRosStream();
    break;
  }
  ros::spinOnce();

  return 0;
}
