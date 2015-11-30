#include <ros/ros.h>
#include <brick_check/check_brick.h>

#include <cv.h>
#include <highgui.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <mutex>

std::mutex mtx;

cv::Mat input_img_;

void imageCB(const sensor_msgs::ImageConstPtr& msg){
    try{
        mtx.lock();
        input_img_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        mtx.unlock();
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

bool checkBrick(brick_check::check_brick::Request &req, brick_check::check_brick::Response &res){
    ROS_INFO("Inside the callback");
    mtx.lock();
    cv::Rect ROI(input_img_.cols/2-120-10, input_img_.rows-150, 240, 150);
    cv::Mat region = input_img_(ROI);

    cv::Rect FrobROI(1020,440,50,50);
    cv::Mat FrobGreyScale, FrobThres;
    cv::Mat Frobregion = input_img_(FrobROI);

    //cv::imshow("testregion", Frobregion);

    cv::cvtColor(Frobregion, FrobGreyScale, CV_BGR2GRAY);
    //cv::imshow("testGray", FrobGreyScale);
    cv::threshold(FrobGreyScale, FrobThres, 100, 255, cv::THRESH_BINARY);
    //cv::imshow("testThres", FrobThres);

    double total = FrobThres.cols * FrobThres.rows;
    double Floor_proportion = cv::countNonZero(FrobThres) / total;
    ROS_INFO("floor: %f" ,Floor_proportion);
    //cv::waitKey();
    cv::Mat hsv, red, blue, yellow;
    std::vector<cv::Mat> channels(3);
    cv::cvtColor(region, hsv, CV_BGR2HSV);
    cv::split(hsv, channels);



    cv::threshold(channels[0], red, 10, 255, cv::THRESH_BINARY_INV);
    cv::threshold(channels[0], blue, 100, 255, cv::THRESH_BINARY);
    cv::threshold(channels[0], yellow, 35, 255, cv::THRESH_BINARY_INV);
    cv::threshold(yellow, yellow, 60, 255, cv::THRESH_BINARY);

    total = channels[0].cols * channels[0].rows;
    double red_proportion = cv::countNonZero(red) / total;
    double blue_proportion = cv::countNonZero(blue) / total;
    double yellow_proportion = cv::countNonZero(yellow) / total;
    mtx.unlock();

    if(Floor_proportion > 0.2 && req.type == 3)
    {
        res.picked = true;
        return true;
    }
    else  if(req.type == 3){
        res.picked = false;
        return true;
    }

    if(red_proportion > 0.2 && req.type == 0){
        res.picked = true;
        return true;
    }
    else if(req.type == 0){
        res.picked = false;
        return true;
    }

    if(blue_proportion > 0.2 && req.type == 1){
        res.picked = true;
        return true;
    }
    else if(req.type == 1){
        res.picked = false;
        return true;
    }

    if(yellow_proportion > 0.2 && req.type == 2){
        res.picked = true;
        return true;
    }
    else if(req.type == 2){
        res.picked = false;
        return true;
    }

    return false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "brick_check_server");
    ros::NodeHandle n("~");

    image_transport::ImageTransport it(n);
    std::string camera_topic;
    n.param("camera_topic", camera_topic, std::string("/usb_cam/image_raw"));
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe(camera_topic, 1, &imageCB);

    ros::ServiceServer service = n.advertiseService("checkBrick", checkBrick);

    ros::spin();
    return 0;
}
