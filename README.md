# rsd3_ROS_workcell_stack
Repository for ROS nodes for the KukaWorkCell, and the RobWork Plugin

## RobWorkRunFiles
Contains run script launching nodes and RobWorkStudio(also load workcell and plugin)

## KR6RSDWorkCell
The Workcell for RobWorkStudio

##KukaROS
Node that handels communication with workcell PC (controlling the Kuka)

##WSG50
Node that handels communication with the WSG50 gripper

##brick_detection
Service that handles the images processing (also launches a camera node that uses video/device1)

##brick_client
Test node for brick_detection service

##rsd_plugin
Plugin for robworkstudio 
Should be build with qt-creater! (using catkin_make can result in old versions of the UI header file (so dont use the plugin located at catkinws/devel/lib)

1. source /catkinws/devel/setup.bash

2. run qtcreator

3. open project /src/rsd_plugin/CMakeLists.txt

4. set the build folder to  /src/rsd_plugin-build 

(do not add new files generated in this folder to the git repository!, only commit changes in ui_rsdPlugin.h)
hint: ui_rsdPlugin.h can be copyed to the rsd_plugint/src folder, if catkin_make generates errors.

The newset plugin compiled by qtcreator will be located in /src/rsd_plugin-build/devel/lib

##rsd_plugin-build
output files of rsd_plugin, when building with qt-creator

##robot_camera_launch
Obsolete camera node
