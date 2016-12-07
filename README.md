# fw_swm_ros
ROS interface for SHERPA World Model (SWM). 

* Inputs data such as human detections, robot poses, raw images, digital elevation model (DEM), orthomosaic.
* Two modes: rosbag, rosstream

# Installing
Create a catkin workspace (if not existing already):
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/
catkin init
```
Install the dependencies:
```
cd src/
git clone git@github.com:ethz-asl/fw_swm_ros.git
wstool init
wstool merge fw_swm_ros/install/dependencies.rosinstall
wstool update
```
Compile:
```
catkin build fw_swm_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

```

# Important links

Sherpa World Model (SWM): <br>
https://github.com/blumenthal/ubx_robotscenegraph

Mediator: <br>
https://github.com/maccradar/sherpa-com-mediator

Functionality in SWM:
- Human detection: https://github.com/blumenthal/ubx_robotscenegraph/blob/master/examples/zyre/swmzyre.h#L74
- Add agent: https://github.com/blumenthal/ubx_robotscenegraph/blob/master/examples/zyre/swmzyre.h#L76
- Update pose: https://github.com/blumenthal/ubx_robotscenegraph/blob/master/examples/zyre/swmzyre.h#L78

# Acknowledgment
This work was supported by the European FP7 project SHERPA (FP7-600958).

# Author
Written by Timo Hinzmann (hitimo@ethz.ch)
