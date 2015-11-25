#!/usr/bin/env python

import sys
import rospy
from plc_comm.srv import plc_service 

def client():
    rospy.loginfo("Launchig PLC client")
    activate = False

    forward = False      
    speed = 2           # Speed 1: 50Hz, Speed 2: 35Hz
    plc_service_client(activate, forward, speed)


def plc_service_client(action, direction, speed):
    rospy.wait_for_service('operate_PLC')
    try:
        send_plc_command = rospy.ServiceProxy('operate_PLC', plc_service)
        reply = send_plc_command(action, direction, speed)
        status = reply.activated
        print status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":

    try:
        client()
    except rospy.ROSInterruptException:
        pass