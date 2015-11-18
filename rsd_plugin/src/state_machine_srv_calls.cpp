#include "state_machine_srv_calls.h"

bool state_machine_srv_calls::openGripper()
{
    bool res = true;
    wsg_50_common::Move srv;
    srv.request.speed = 100;
    srv.request.width = 40;
    //ROS_INFO("Gripper trying to move to width =%d at speed=%d", (int)srv.request.width,(int)srv.request.speed);
    if (ros::service::call("/wsg_50/move",srv))
    {
      if(srv.response.error > 0) {
          //error from gripper... try ack and move again
          /*log().info() << "Gripper error: " << (int)srv.response.error << "\n" <<
                          "Acknowledgeing and trying to open the gripper again..." << "\n";*/
          std_srvs::Empty AckSrv;
          ros::service::call("/wsg_50/ack",AckSrv);
          if (ros::service::call("/wsg_50/move",srv))
          {
              if(srv.response.error > 0) {
                  //log().info() << "Gripper error: " << (int)srv.response.error << "\n";
                  return false;
              }
              else {
                   //log().info() << "Gripper moved to width = " << (int)srv.request.width << " at speed=" << (int)srv.request.speed << "\n";
                   return true;
              }
          }
          return false;
      } else {
      //   log().info() << "Gripper open" << "\n";
      }
   }
    else
    {
        res = false;
    }
    return res;
}

bool state_machine_srv_calls::closeGripper(bool BrickOnSide, double speedPct)
{
    bool res = true;
    wsg_50_common::Move srv;
    srv.request.speed = speedPct;
    if(BrickOnSide)
        srv.request.width = 30.0;//not calibrated
    else
        srv.request.width = 22.0;
    //ROS_INFO("Gripper trying to move to width =%d at speed=%d", (int)srv.request.width,(int)srv.request.speed);
    if (ros::service::call("/wsg_50/move",srv))
    {
      if(srv.response.error > 0) {
          //error from gripper... try ack and move again
       //   log().info() << "Gripper error: " << (int)srv.response.error << "\n" <<
       //                   "Acknowledgeing and trying to close the gripper again..." << "\n";
          std_srvs::Empty AckSrv;
          ros::service::call("/wsg_50/ack",AckSrv);
          if (ros::service::call("/wsg_50/move",srv))
          {
              if(srv.response.error > 0) {
        //          log().info() << "Gripper error: " << (int)srv.response.error << "\n";
                  return false;
              }
              else {
         //          log().info() << "Gripper moved to width = " << (int)srv.request.width << " at speed=" << (int)srv.request.speed << "\n";
                   return true;
              }
          }
          return false;
      } else {
      //     log().info() << "Gripper moved to width = " << (int)srv.request.width << " at speed=" << (int)srv.request.speed << "\n";
           return true;
      }
   }
    else
    {
        return false;
    }
    return res;
}

bool state_machine_srv_calls::brickPresent(int color)
{
    brick_detection::bricks _brick_srv;
    ros::service::call("/brick_detector/getBricks",_brick_srv);
    if(_brick_srv.response.x.size() > 0)
    {
        for(unsigned int i = 0; i !=_brick_srv.response.x.size(); i++)
        {
            if(color == _brick_srv.response.type[i])
                return true;
        }
    }
    return false;
}

bool state_machine_srv_calls::robotMoving()
{
    kuka_ros::getIsMoving _isMoving_srv;
    ros::service::call("/KukaNode/IsMoving",_isMoving_srv);
    if(_isMoving_srv.response.isMoving)
        return true;
    else
        return false;
}

