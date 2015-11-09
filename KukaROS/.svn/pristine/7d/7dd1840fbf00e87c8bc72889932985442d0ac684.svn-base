// Includes
#include <ros/ros.h>
#include "kuka_ros/getConfiguration.h"
#include "kuka_ros/setConfiguration.h"
#include "kuka_ros/getQueueSize.h"
#include "kuka_ros/getIsMoving.h"
#include "kuka_ros/stopRobot.h"
#include "kuka_ros/getSafety.h"
#include "std_msgs/String.h"
#include <iostream>
#include "PracticalSocket.h"

// Defines
#define TCP_IP_SERVER           "192.168.100.50"
#define TCP_PORT_SERVER         49002
#define DEBUG                   false
#define DEF_PRECISION           5

// Global variables
TCPSocket *client;

// Functions
bool isDigitsOnly(const std::string &str)
{
    return str.find_first_not_of("0123456789") == std::string::npos;
}

bool setConfigurationCallback(kuka_ros::setConfiguration::Request &req, kuka_ros::setConfiguration::Response &res)
{
    // Add Q
    std::stringstream str;
    str << "SetConf:Q{";

    for(unsigned int i=0; i<req.q.size(); i++)
    {
        str << std::fixed << std::setprecision(DEF_PRECISION) << req.q[i];

        if(i<req.q.size()-1)
            str << ", ";
    }
    str << "},V{";

    // Add Speed
    for(unsigned int i=0; i<req.speed.size(); i++)
    {
        str << std::fixed << std::setprecision(DEF_PRECISION) << req.speed[i];

        if(i<req.speed.size()-1)
            str << ", ";
    }
    str << "}";

    client->send(str.str());

    if(DEBUG)
        std::cout << str.str() << std::endl;

    return true;
}

bool getConfigurationCallback(kuka_ros::getConfiguration::Request &req, kuka_ros::getConfiguration::Response &res)
{
    client->send("GetConf");
    std::string msg = client->recv(100);

    if(msg.empty())
    {
        ROS_ERROR("Error: Got empty msg!");
        return false;
    }
    else
    {
        if(DEBUG)
            std::cout << msg << std::endl;

        // Isolate data
        int i = 0;
        while(msg[i++] != '{');
        msg = msg.substr(i, msg.size()-i);
        i = 0;
        while(msg[i++] != '}');
        msg = msg.substr(0, i-1);

        // Get data
        std::stringstream ss(msg);
        double d = 0;
        i = 0;
        while(ss >> d)
        {
            res.q[i++] = d;
            if(ss.peek() == ',' || ss.peek() == ' ')
                ss.ignore();
        }

        return true;
    }
}

bool getQueueSizeCallback(kuka_ros::getQueueSize::Request &req, kuka_ros::getQueueSize::Response &res)
{
    client->send("GetQueueSize");
    std::string msg = client->recv(100);

    if(msg.empty())
    {
        ROS_ERROR("Error: Got empty msg!");
        return false;
    }
    else
    {
        if(DEBUG)
            std::cout << msg << std::endl;

        if(isDigitsOnly(msg))
        {
            // Handle msg
            res.queueSize = atoi(msg.c_str());
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool stopRobotCallback(kuka_ros::stopRobot::Request &req, kuka_ros::stopRobot::Response &res)
{
    client->send("StopRobot");
    /*std::string msg = client->recv(100);

    if(msg.empty())
    {
        ROS_ERROR("Error: Got empty msg!");
        return false;
    }
    else
    {
        if(DEBUG)
            std::cout << msg << std::endl;

        // Handle msg
        if(msg == "Stopped")
            return true;
        else
            return false;
    }*/
}

bool isMovingCallback(kuka_ros::getIsMoving::Request &req, kuka_ros::getIsMoving::Response &res)
{
    client->send("IsMoving");
    std::string msg = client->recv(100);

    if(msg.empty())
    {
        ROS_ERROR("Error: Got empty msg!");
        return false;
    }
    else
    {
        if(DEBUG)
            std::cout << msg << std::endl;

        // Handle msg
        if(msg == "true")
        {
            res.isMoving = true;
            return true;
        }
        else if(msg == "false")
        {
            res.isMoving = false;
            return true;
        }
        else
            return false;
    }
}

bool safetyCallback(kuka_ros::getSafety::Request &req, kuka_ros::getSafety::Response &res)
{
    client->send("GetSafety");
    std::string msg = client->recv(100);

    if(msg.empty())
    {
        ROS_ERROR("Error: Got empty msg!");
        return false;
    }
    else
    {
        if(DEBUG)
            std::cout << msg << std::endl;

        // Handle msg
        if(msg == "false")
        {
            res.safetyBreached = true;
            return true;
        }
        else if(msg == "true")
        {
            res.safetyBreached = false;
            return true;
        }
        else
            return false;
    }
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "Kuka_ROS");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // Topic names
    std::string servicePrefixName, serverIP;
    int serverPort;
    pNh.param<std::string>("CmdServiceName", servicePrefixName, "/KukaNode");
    pNh.param<std::string>("ServerIP", serverIP, TCP_IP_SERVER);
    pNh.param<int>("ServerPort", serverPort, TCP_PORT_SERVER);

    // Create service handlers
    ros::ServiceServer getConfigurationService = nh.advertiseService(servicePrefixName + "/GetConfiguration", getConfigurationCallback);
    ros::ServiceServer setConfigurationService = nh.advertiseService(servicePrefixName + "/SetConfiguration", setConfigurationCallback);
    ros::ServiceServer getQueueSizeService = nh.advertiseService(servicePrefixName + "/GetQueueSize", getQueueSizeCallback);
    ros::ServiceServer stopRobotService = nh.advertiseService(servicePrefixName + "/StopRobot", stopRobotCallback);
    ros::ServiceServer isMovingService = nh.advertiseService(servicePrefixName + "/IsMoving", isMovingCallback);
    ros::ServiceServer getSafetyService = nh.advertiseService(servicePrefixName + "/GetSafety", safetyCallback);

    // Setup TCP Client
    client = new TCPSocket(serverIP, serverPort);

    // Inform
    ROS_INFO("Successfully connected to server!");

    // Sleep rate
    ros::Rate r(100);

    // Set loop rate
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    // Return
    client->closeConnection();
    return 0;
}
