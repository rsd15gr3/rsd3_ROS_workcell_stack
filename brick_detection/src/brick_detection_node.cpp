#include <stdio.h>
#include <ros/ros.h>
#include <brick_detection/bricks.h>
#include "detectBricks.h"


#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>
#include <mutex>


using namespace cv;
using std::vector;
using std::string;

Mat newest_image;
std::mutex mtx;

void imageCb(const sensor_msgs::ImageConstPtr& msg);

image_transport::Publisher image_pub;

string camera_frame_id;

bool findBricks(brick_detection::bricks::Request  &req,
                brick_detection::bricks::Response &res)
{
    ROS_INFO("got a request");

    mtx.lock();
    ROS_INFO("Locked mutex");
    //Rect ROI(525 ,305,newest_image.cols-1-1045,newest_image.rows-435);
    Rect ROI(600,230,720,620);    
    Mat img = newest_image(ROI);
    //ROS_INFO("img size: %d x %d",newest_image.cols, newest_image.rows);
    //cv::imshow("test", img);
    //cv::waitKey(1);

    ROS_INFO("Got ROI mat");
    std::vector<brick> bricks=detectBrick(img);
    ROS_INFO("Done looking for bricks");

    

    //res.angle.resize(1);	
    for(int i=0; i<bricks.size(); i++)
    {	
	std::cout<<"Found "<<bricks[i].color<<" brick located at ("<<bricks[i].center.x<<","<<bricks[i].center.y<<") with orientation "<<bricks[i].orientation<<" radians."<<std::endl;
        res.x.push_back(bricks[i].center.x);
        res.y.push_back(bricks[i].center.y);
        res.angle.push_back(bricks[i].orientation);
	if(bricks[i].color=="red"){        
		res.type.push_back(0);
	}
	if(bricks[i].color=="yellow"){        
		res.type.push_back(1);
	}
	if(bricks[i].color=="blue"){        
		res.type.push_back(2);
	}
    }

    mtx.unlock();
    ROS_INFO("returning true");
    if(bricks.size()> 0)
        return true;
    else
        return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brick_detector_servor");
    ros::NodeHandle n("~");

    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/usb_cam/image_raw", 1, &imageCb);

    ros::ServiceServer service = n.advertiseService("getBricks", findBricks);

    ros::spin();
    return 0;
}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    try
    {
        cv_ptr =  cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mtx.lock();
    newest_image = cv_ptr->image.clone();
    //cv::waitKey(1);
    mtx.unlock();

    //cv::imshow("test2", newest_image);
}

