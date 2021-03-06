#!/bin/bash
#Make sure the catkin_ws is sourced
source ../../devel/setup.sh
#Launch the robot camera node in a new terminal (also start roscore if it not running)
#xterm -e roslaunch robot_camera_launch robo_cam.launch & #brick detection launched the camera
export ROS_IP=$(hostname -I | awk '{print $1}') #set ROS_IP to the current wifi IP
xterm -e roslaunch brick_detection brick_detection.launch &
#Sleep a bit (Roscore need to start before we proceed)
sleep 5s
#Launch rest of the nodes in new terminals
xterm -e roslaunch kuka_ros kukaros.launch & #Kuka node
xterm -e rosrun brick_check brick_check_node & #check brick node
xterm -e roslaunch ../WSG50/launch/wsg_50_tcp.launch & #gripper node
xterm -e rosrun plc_comm PLC_server.py & #plc node
#hmi stuff
python IPs_synchronization/publish_my_ip_as.py workcell #"upload" computer ip for hmi
xterm -e roslaunch rosbridge_server rosbridge_websocket.launch & #rosbridge_websocket node
xterm -e roslaunch hmi_support_workcell hmi_support_workcell.launch & #workcell hmi support node
#Wait a bit (let rest of the nodes start before we start RobWorkStudio)
sleep 2s
#Start robworkstudioand open the SceneKu6 (also locates RobWorkStudio.ini, and starts the plug in defined there)
${RWS_ROOT}/bin/release/RobWorkStudio "../KR6RSDWorkCell/SceneKu6.wc.xml"
