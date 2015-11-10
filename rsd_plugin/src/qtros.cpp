#include "qtros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>

#define SUBSCRIBER "/usb_cam/image_raw"

QtROS::QtROS(): _it(_nh) {
  ROS_INFO("Connected to roscore");
  _image_sub = _it.subscribe(SUBSCRIBER, 1, &QtROS::imageCallback, this);

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
