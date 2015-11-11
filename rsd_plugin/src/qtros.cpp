#include "qtros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>

#define SUBSCRIBER "/usb_cam/image_raw"

QtROS::QtROS(): _it(_nh) {
  ROS_INFO("Connected to roscore");
  _image_sub = _it.subscribe(SUBSCRIBER, 1, &QtROS::imageCallback, this);
   setConfigurationService = _nh.advertiseService("/rsdPlugin/SetConfiguration", &QtROS::setConfigurationCallback, this);
   manualControl_sub = _nh.subscribe("/wc_automode", 1, &QtROS::manualControlCallback, this);
   //_q_client = _nh.serviceClient<kuka_ros::getConfiguration>("/GetConfiguration");
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

void QtROS::manualControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(autoControlEnabled != msg->data)
    {
        autoControlEnabled = msg->data;
        emit autoControlEnabledSignal(!autoControlEnabled); //requst statemachine for manual control
    }
}

void QtROS::setConfigurationAuto(kuka_ros::setConfiguration _q_srv)
{
    if(autoControlEnabled)
        ros::service::call("/KukaNode/SetConfiguration",_q_srv);
    else
        ROS_ERROR("Ignoring configuration! (system is on auto!)");
}

bool QtROS::setConfigurationCallback(kuka_ros::setConfiguration::Request &req, kuka_ros::setConfiguration::Response &res)
{
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

void QtROS::process()
{
   //emit newImage(cv_ptr->image);
}

void QtROS::run(){ 
  while(ros::ok() && !quitfromgui) {
    ros::spinOnce(); 

    if (ros::service::call("/KukaNode/GetConfiguration",_q_srv))
               emit updateConfiguration(_q_srv);
    
    ros::Duration(1./10.).sleep();
    }
  if (!quitfromgui) {
    emit rosQuits();
    ROS_INFO("ROS-Node Terminated\n"); 
}
}
//http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
