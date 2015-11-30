#include "qtros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>

#define SUBSCRIBER "/usb_cam/image_raw"

QtROS::QtROS(): _it(_nh) {
  ROS_INFO("Connected to roscore");
  //_image_sub = _it.subscribe(SUBSCRIBER, 1, &QtROS::imageCallback, this);
   setConfigurationService = _nh.advertiseService("/rsdPlugin/SetConfiguration", &QtROS::setConfigurationCallback, this);
   manualControl_sub = _nh.subscribe("/wc_automode", 1, &QtROS::manualControlCallback, this);
   order_sub = _nh.subscribe("/wc_order", 1,&QtROS::orderCallback,this); ///NEEDS TESTING
  quitfromgui = false; }

void QtROS::quitNow(){ 
  quitfromgui = true; }

void QtROS::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    _imageIn = msg;
    try{
     _imageOut = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     emit newImage(_imageOut->image);
    }
    catch(cv_bridge::Exception& e)
    {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
    }
}

void QtROS::orderCallback(const std_msgs::Bool::ConstPtr& msg) //change msg type
{
  ///get order
    emit newOrder();
}

void QtROS::manualControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(autoControlEnabled != msg->data)
    {
        autoControlEnabled = msg->data;
        emit autoControlEnabledSignal(!autoControlEnabled); //requst statemachine for manual control
    }
}

//Autonom movement
void QtROS::setConfigurationAuto(kuka_ros::setConfiguration _q_srv)
{
    //check if autocontrol is enabled (if system is on manual, this callback must not move the robot)
    if(autoControlEnabled)
        ros::service::call("/KukaNode/SetConfiguration",_q_srv);
    else
        ROS_ERROR("Ignoring configuration! (system is on manual!)");
}

///Set configuration callback for
bool QtROS::setConfigurationCallback(kuka_ros::setConfiguration::Request &req, kuka_ros::setConfiguration::Response &res)
{
    //check if autocontrol is enabled (if system is on auto, this callback must not move the robot)
    if(!autoControlEnabled)
    {
        kuka_ros::setConfiguration _UI_srv;
        _UI_srv.request = req;
        ros::service::call("/KukaNode/SetConfiguration",_UI_srv);
        return true;
    }
    else
    {
        ROS_ERROR("Ignoring configuration! (system is on auto!)");
        return false;
    }
}

void QtROS::run(){ 
  while(ros::ok() && !quitfromgui) {
    ros::spinOnce(); 

    //if (ros::service::call("/KukaNode/GetConfiguration",_q_srv))
      //         emit updateConfiguration(_q_srv);
    
    ros::Duration(1./5.).sleep();
    }
  if (!quitfromgui) {
    emit rosQuits();
    ROS_INFO("ROS-Node Terminated\n"); 
}
}
//http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
