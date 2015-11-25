#!/usr/bin/env python


from plc_comm.srv import plc_service 
import rospy
import sys
import serial 
import os

# Activa/deactivate outputs of PLC
def handlePLCOutput(commObj, outputChannel, action):
    command = "input"
    args = ["", "", ""]
    args[0] = "0" + "%d" % outputChannel
    args[1] = "is"
    args[2] = "%s" % action
    for i in args:
        command = command + " " + i

    print command
    commObj.write(command)
    readPLCresponse(commObj)
    rospy.sleep(0.05) # sleep for 0.1 seconds

# Read inputs of PLC
def handlePLCInput(commObj, inputChannel):
    command = "read"
    args = [""]
    args[0] = "0" + "%d" % inputChannel
    for i in args:
        command = command + " " + i

    print command
    commObj.write(command)
    readPLCresponse(commObj)

#TODO: handle failures in the PLC reception
# Reply from PLC: r = success
#                 x = failure
def readPLCresponse(commObj):
    line = commObj.readline()       # read a '\n' terminated line
    print line


# Activates the predefined speed in the motor drive (1 and 2)
# Makes sure the two of them are not activated at the same time
def switchSpeed(predefSpeed, ser):
    if predefSpeed == 1:
        handlePLCOutput(ser, 3, "False")
        handlePLCOutput(ser, 2, "True")
    else:
        handlePLCOutput(ser, 2, "False")
        handlePLCOutput(ser, 3, "True") 


def plc_serv_callback(request):
    # Opening and configuring serial port
    ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
    if ser.isOpen() == False:
        rospy.loginfo('Could not open the port')
        rospy.ROSInterruptException

    # Request for stopping the conveyor
    if request.activate == False:
        print "stop conveyor"
    	handlePLCOutput(ser, 0, "False")
    	return False 
    # Request for activating the conveyor: four possibilities (direction + two predefined speeds)
    else: 
        print "start conveyor"
        switchSpeed(request.preDefSpeed, ser)       # Choose predef speed
        handlePLCOutput(ser, 1, request.direction)  # Config direction 
        handlePLCOutput(ser, 0, "True")             # Start conveyor
    
    inputs = [0, 1]
    ser.flushInput()
    for i in inputs:
        rospy.sleep(0.5) # sleep for 0.1 seconds
        handlePLCInput(ser, i)

    ser.close()
    return True 


def server():
    rospy.init_node('plc_comm_node')
    rospy.loginfo('Launching PLC server')
    s = rospy.Service('operate_PLC', plc_service, plc_serv_callback)
    rospy.spin()

if __name__ == "__main__":

    try:
        server()
    except rospy.ROSInterruptException:
        pass
