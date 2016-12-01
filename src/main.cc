#include <Eigen/Dense>
#include <gflags/gflags.h>
#include <iostream>
#include <memory> // shared_ptr
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

extern "C" {
#include "/usr/local/include/swmzyre.h"
}

DEFINE_string(config_filename, "/home/timo/catkin_ws/src/swm_ros/src/swm_zyre_config.json",
              "Path and filename of swm_zyre_config.json");

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "swm_zyre");
  ros::Time::init();

  sensor_msgs::NavSatFix human_detection;
  human_detection.latitude = 42.0;
  human_detection.longitude = 14.0;
  human_detection.altitude = 415.0;
  human_detection.header.stamp = ros::Time::now();

  // Load configuration file for communication setup.
  char* config_filename = FLAGS_config_filename;
  json_t* config = load_config_file(config_filename);
  if (config == NULL) {
    return -1;
  }

  // Spawn new communication component.
  component_t* self = new_component(config);
  if (self == NULL) {
      return -1;
  }
  printf("[%s] component initialized!\n", self->name);

  VLOG(1) << "Add victims to SWM ..." << std::endl;

  // "Stress test".
  for (size_t i = 0u; i < 100; ++i) {
    add_victim(self, human_detection.latitude,
               human_detection.longitude,
               human_detection.altitude,
               human_detection.header.stamp.toSec() * 1.0e3,
               "hawk");
  }
  VLOG(1) << "Done!" << std::endl;
  free(config_filename);
  free(config);

  return 0;
}
