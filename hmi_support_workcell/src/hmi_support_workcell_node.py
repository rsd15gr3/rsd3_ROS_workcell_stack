#!/usr/bin/env python

"""
This node enables the HMI to control Workcell and Conveyor Belt

subscribing:
- ui/str_control_workcell

publishing:
- /ui/wc_automode   (20Hz, Workcell's automode state, BoolStamped)
- /ui/belt_automode (20Hz, Belt's automode state, BoolStamped)
- /ui/belt_activated (On Manual Control Change, Belt's activated state, BoolStamped)
- /ui/belt_forward (On Manual Control Change, Belt's forward direction state, BoolStamped)
- /ui/belt_speed (On Manual Control Change, Belt's speed value, IntStamped, 1 ... fast, 2 ... slow)

calling services:
- /operate_PLC (Control of Conveyor Belt)

"""

import rospy
from std_msgs.msg import String, Bool, Int8
from plc_comm.srv import plc_service as plc_service_type


class WorkcellHMI():

    def __init__(self, node):
        """ Workcell HMI Support Instance Initialization """

        ''' Register self '''
        self.node = node
        
        ''' Read parameters from launchfile '''
        self.tp_automode = rospy.get_param('~wc_tp_automode', '/ui/wc_automode')

        ''' Setup topics '''
        # Setup Workcell automode publish topic
        self.tp_automode_message = Bool()
        self.tp_automode_message.data = False
        self.tp_automode_publisher = rospy.Publisher(self.tp_automode, Bool, queue_size=1)

    def decode_control(self, data):
        if data[1] == 'mode':
            if data[2] == 'auto':
                self.tp_automode_message.data = True
            elif data[2] == 'manual':
                self.tp_automode_message.data = False

    def publish_tp_automode_message(self):
        self.tp_automode_publisher.publish(self.tp_automode_message)


class BeltHMI():

    def __init__(self, node):
        """ Belt HMI Support Instance Initialization """

        ''' Register self '''
        self.node = node

        ''' Read parameters from launchfile '''
        self.tp_automode = rospy.get_param('~belt_tp_automode', '/ui/belt_automode')
        self.tp_activated = rospy.get_param('~belt_tp_activated', '/ui/belt_activated')
        self.tp_forward = rospy.get_param('~belt_tp_forward', '/ui/belt_forward')
        self.tp_speed = rospy.get_param('~belt_tp_speed', '/ui/belt_speed')
        self.srv_operate_plc = rospy.get_param('~belt_srv_operate_name', 'operate_PLC')
        self.srv_operate_plc_timeout = rospy.get_param('~belt_srv_operate_timeout', None)
        
        ''' Variables '''
        self.automode = False
        self.activated = False
        self.forward = True
        self.speed = 1
        
        ''' Setup topics '''
        # Setup Belt automode publish topic
        self.tp_automode_message = Bool()
        self.tp_automode_message.data = self.automode
        self.tp_automode_publisher = rospy.Publisher(self.tp_automode, Bool, queue_size=1)
        
        # Setup Belt activated publish topic
        self.tp_activated_message = Bool()
        self.tp_activated_message.data = self.activated
        self.tp_activated_publisher = rospy.Publisher(self.tp_activated, Bool, queue_size=1)
        
        # Setup Belt forward publish topic
        self.tp_forward_message = Bool()
        self.tp_forward_message.data = self.forward
        self.tp_forward_publisher = rospy.Publisher(self.tp_forward, Bool, queue_size=1)
        
        # Setup Belt speed publish topic
        self.tp_speed_message = Int8()
        self.tp_speed_message.data = self.speed
        self.tp_speed_publisher = rospy.Publisher(self.tp_speed, Int8, queue_size=1)

    def decode_control(self, data):
        if data[1] == 'srv':
            self.automode = False
            if data[2] == 'on':
                self.activated = True
            elif data[2] == 'off':
                self.activated = False
            if data[3] == 'forward':
                self.forward = True
            elif data[3] == 'backwards':
                self.forward = False
            if data[4] == 'slow':
                self.speed = 2
            elif data[4] == 'fast':
                self.speed = 1
            self.send_control()
        elif data[1] == 'mode':
            if data[2] == 'auto':
                self.automode = True
            elif data[2] == 'manual':
                self.stop_belt()
                self.automode = False
        else:
            print 'This structure of message is not yet implemented to be processed:', data

    def stop_belt(self):
        self.activated = False
        self.send_control()

    def send_control(self):
        rospy.loginfo("Launching PLC client")
        try:
            rospy.wait_for_service(self.srv_operate_plc, timeout=self.srv_operate_plc_timeout)
            try:
                service = rospy.ServiceProxy(self.srv_operate_plc, plc_service_type)
                reply = service(self.activated, self.forward, self.speed)
                print 'PLC Service Command Status:', reply.activated
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        except rospy.ROSException:
            print "PLC Service unreachable!"
        self.publish_belt_info()

    def publish_belt_info(self):
        self.publish_tp_activated_message()
        self.publish_tp_forward_message()
        self.publish_tp_speed_message()

    def publish_tp_automode_message(self):
        self.tp_automode_message.data = self.automode
        self.tp_automode_publisher.publish(self.tp_automode_message)
        
    def publish_tp_activated_message(self):
        self.tp_activated_message.data = self.activated
        self.tp_activated_publisher.publish(self.tp_activated_message)
        
    def publish_tp_forward_message(self):
        self.tp_forward_message.data = self.forward
        self.tp_forward_publisher.publish(self.tp_forward_message)
        
    def publish_tp_speed_message(self):
        self.tp_speed_message.data = self.speed
        self.tp_speed_publisher.publish(self.tp_speed_message)


class Node():

    def __init__(self):
        """ Node Instance Initialization """

        ''' Init Workcell's functionalities '''
        self.wc = WorkcellHMI(node=self)

        ''' Init Belt's functionalities '''
        self.belt = BeltHMI(node=self)

        ''' Init /ui/str_control_workcell topic '''
        self.tp_ui_str_control = rospy.get_param('~tp_ui_str_control', '/ui/str_control_workcell')
        rospy.Subscriber(self.tp_ui_str_control, String, self.on_topic_ui_str_control)

        ''' Publishing rate '''
        self.publishing_rate = rospy.Rate(rospy.get_param('~publishing_rate', 20))          # [Hz]

        # Loop function
        self.keep_publishing()

    def on_topic_ui_str_control(self, msg):
        data = msg.data.split('_')
        if data[0] == 'wc':
            self.wc.decode_control(data=data)
        elif data[0] == 'belt':
            self.belt.decode_control(data=data)

    def keep_publishing(self):
        while not rospy.is_shutdown():
            self.belt.publish_tp_automode_message()
            self.wc.publish_tp_automode_message()
            self.publishing_rate.sleep()

# main function
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('hmi_support_workcell_node')
    try:
        Node()
    except rospy.ROSInterruptException:
        pass


