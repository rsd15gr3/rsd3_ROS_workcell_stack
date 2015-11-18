/** 
 * QtThread based class encapsulating the ros basics,
 * i.e., init, node handle creation, spining and quitting.
 * To quit via qt, connect the quitNow slot to, e.g., 
 * the aboutToQuit-Signal of qapplication.
 */
#ifndef QT_ROS_H
#define QT_ROS_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "kuka_ros/getConfiguration.h"
#include "kuka_ros/setConfiguration.h"
#include <std_msgs/Bool.h>
#include <QThread>
#include <QObject>

class QtROS : public QThread {
  Q_OBJECT

  public:
    ///Note: The constructor will block until connected with roscore
    ///Instead of ros::spin(), start this thread with the start() method
    ///to run the event loop of ros
    QtROS();
    //ros::NodeHandle getNodeHandle(){ return *n; }
    /// This method contains the ROS event loop. Feel free to modify 
    void run();
  public slots:
    ///Connect to aboutToQuit signals, to stop the thread
    void setConfigurationAuto(kuka_ros::setConfiguration _q_srv); //signal from auto
    void quitNow();
  signals:
    ///Triggered if ros::ok() != true
    void rosQuits();
    void autoControlEnabledSignal(bool); //requst for manual control
    void newImage(cv::Mat);
    void updateConfiguration(kuka_ros::getConfiguration);
  private:
    bool autoControlEnabled = true;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void manualControlCallback(const std_msgs::Bool::ConstPtr& msg);
    bool setConfigurationCallback(kuka_ros::setConfiguration::Request &req, kuka_ros::setConfiguration::Response &res);
    bool quitfromgui;
    sensor_msgs::ImageConstPtr _imageIn;
    cv_bridge::CvImagePtr _imageOut;
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;
    ros::Subscriber manualControl_sub;
    ros::ServiceClient _q_client;
    ros::ServiceServer setConfigurationService;
    kuka_ros::getConfiguration _q_srv;

};
#endif

