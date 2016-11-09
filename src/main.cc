#include <string>
#include <iostream>
#include <gflags/gflags.h>
#include <memory> // shared_ptr
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

extern "C" {
#include "/usr/local/include/swmzyre.h"
}

DEFINE_bool(use_brisk, true, "SWE backend enabled/disabled");

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "swm_zyre");
  ros::Time::init();
  std::cout << "SHERPA SWM TEST" << std::endl;

  sensor_msgs::NavSatFix human_detection;
  human_detection.latitude = 42.0;
  human_detection.longitude = 14.0;
  human_detection.altitude = 415.0;
  human_detection.header.stamp = ros::Time::now();

  std::cout << "add victim" << std::endl;
  std::string author = "hawk";
  char *result = strdup(author.c_str());

  /* Load configuration file for communication setup */
  char *config_name = "/home/timo/catkin_ws/src/swm_ros/src/swm_zyre_config.json";

  for (size_t i = 0u; i < 10; ++i) {
    add_victim(human_detection.latitude,
               human_detection.longitude,
               human_detection.altitude,
               human_detection.header.stamp.toSec() * 1.0e3,
               "hawk", config_name);
  }
  free(result);
  free(config_name);

  return 0;
}
