#include <brick_detection/bricks.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brick_client");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<brick_detection::bricks>("/brick_detector/getBricks");

    brick_detection::bricks::Request  req;
    brick_detection::bricks::Response res;

    ROS_INFO("Makeing a request");

    if(client.call(req, res))
	ROS_INFO("request succes");
    else
	ROS_INFO("request failed");

    return 0;
}
