#!/usr/bin/env python

"""
This node enables the HMI to control Workcell and Conveyor Belt

subscribing:
- ui/str_control_workcell

publishing:
- /ui/wc_automode   (20Hz, Workcell's automode state, BoolStamped)
- /ui/belt_automode (20Hz, Workcell's automode state, BoolStamped)

calling services:
- /rsdPlugin/SetConfiguration (Manual control from HMI)
- /KukaNode/GetConfiguration (Reading current KUKA position)
- /operate_PLC (Control of Conveyor Belt)

"""

import rospy
from std_msgs.msg import String
from msgs.msg import BoolStamped
from ast import literal_eval


class WorkcellHMI():

    def __init__(self, node):
        """ Workcell HMI Support Instance Initialization """

        ''' Register self '''
        self.node = node
        
        ''' Read parameters from launchfile '''
        self.tp_automode = rospy.get_param('~wc_tp_automode', '/ui/wc_automode')
        self.srv_set_conf_name = rospy.get_param('~wc_srv_set_conf_name', '/rsdPlugin/SetConfiguration')
        self.srv_set_conf_type = rospy.get_param('~wc_srv_set_conf_type', 'kuka_ros/setConfiguration')
        self.srv_set_conf_timeout = rospy.get_param('~wc_srv_set_conf_timeout', None)
        self.srv_get_conf_name = rospy.get_param('~wc_srv_get_conf_name', '/KukaNode/GetConfiguration')
        self.srv_get_conf_type = rospy.get_param('~wc_srv_get_conf_type', 'kuka_ros/getConfiguration')
        self.srv_get_conf_timeout = rospy.get_param('~wc_srv_get_conf_timeout', None)
        self.kuka_joints_steps = literal_eval(rospy.get_param('~kuka_joints_steps', '[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]'))

        ''' Setup topics '''
        # Setup Workcell automode publish topic
        self.tp_automode_message = BoolStamped()
        self.tp_automode_message.data = False
        self.tp_automode_publisher = rospy.Publisher(self.tp_automode, BoolStamped, queue_size=1)

        ''' Variables '''
        self.current_kuka_configuration = None

    def decode_control(self, data):
        if data[1] == 'joint':
            self.tp_automode_message.data = False
            joint_ind = int(data[2][1])
            try:
                if data[3] == 'down':
                    self.current_kuka_configuration.q[joint_ind] -= self.kuka_joints_steps[joint_ind]
                elif data[3] == 'up':
                    self.current_kuka_configuration.q[joint_ind] += self.kuka_joints_steps[joint_ind]
            except AttributeError:
                print 'Do not know current KUKA configuration, so can not move it'
            #self.suggest_configuration()
        elif data[1] == 'mode':
            if data[2] == 'auto':
                self.tp_automode_message.data = True
            elif data[2] == 'manual':
                self.stop_workcell()
                self.tp_automode_message.data = False
        else:
            print 'This structure of message is not yet implemented to be processed:', data

    def stop_workcell(self):
        # TODO
        pass

    def get_kuka_configuration(self):
        try:
            rospy.wait_for_service(self.srv_get_conf_name, timeout=self.srv_get_conf_timeout)
            try:
                service = rospy.ServiceProxy(self.srv_get_conf_name, self.srv_get_conf_type)
                self.current_kuka_configuration = service()     # TODO
                print self.current_kuka_configuration
            except rospy.ServiceException:
                self.current_kuka_configuration = None
                print 'Service call (get_configuration) failed.'
        except rospy.ROSException:
            self.current_kuka_configuration = None
            print "WC Manual Control: GetConf Service unreachable!"

    def suggest_configuration(self):
        try:
            rospy.wait_for_service(self.srv_set_conf_name, timeout=self.srv_set_conf_timeout)
            try:
                service = rospy.ServiceProxy(self.srv_set_conf_name, self.srv_set_conf_type)
                reply = service(self.current_kuka_configuration)
                print reply
            except rospy.ServiceException:
                print 'Service call (manual_configuration) failed.'
        except rospy.ROSException:
            print "WC Manual Control: SetConf Service unreachable!"

    def publish_tp_automode_message(self):
        self.tp_automode_message.header.stamp = rospy.get_rostime()
        self.tp_automode_publisher.publish (self.tp_automode_message)


class BeltHMI():

    def __init__(self, node):
        """ Belt HMI Support Instance Initialization """

        ''' Register self '''
        self.node = node

        ''' Read parameters from launchfile '''
        self.tp_automode = rospy.get_param('~belt_tp_automode', '/ui/belt_automode')
        self.srv_operate_name = rospy.get_param('~belt_srv_operate_name', 'operate_PLC')
        self.srv_operate_type = rospy.get_param('~belt_srv_operate_type', 'plc_comm/plc_service')
        self.srv_operate_timeout = rospy.get_param('~belt_srv_operate_timeout', None)

        ''' Setup topics '''
        # Setup Belt automode publish topic
        self.tp_automode_message = BoolStamped()
        self.tp_automode_message.data = False
        self.tp_automode_publisher = rospy.Publisher(self.tp_automode, BoolStamped, queue_size=1)

        ''' Variables '''
        self.activate = False
        self.forward = True
        self.speed = 1

    def decode_control(self, data):
        if data[1] == 'srv':
            self.tp_automode_message.data = False
            if data[2] == 'on':
                self.activate = True
            elif data[2] == 'off':
                self.activate = False
            if data[3] == 'forward':
                self.forward = True
            elif data[3] == 'backwards':
                self.forward = False
            if data[4] == 'slow':
                self.speed = 2
            elif data[4] == 'fast':
                self.speed = 1
            #self.send_control()
        elif data[1] == 'mode':
            if data[2] == 'auto':
                self.tp_automode_message.data = True
            elif data[2] == 'manual':
                #self.stop_belt()
                self.tp_automode_message.data = False
        else:
            print 'This structure of message is not yet implemented to be processed:', data

    def stop_belt(self):
        self.activate = False
        self.send_control()

    def send_control(self):
        rospy.loginfo("Launching PLC client")
        try:
            rospy.wait_for_service(self.srv_operate_name, timeout=self.srv_operate_timeout)
            try:
                plc_service = rospy.ServiceProxy(self.srv_operate_name, self.srv_operate_type)
                reply = plc_service(self.activate, self.forward, self.speed)
                print 'PLC Service Command Status:', reply.activated
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        except rospy.ROSException:
            print "PLC Service unreachable!"

    def publish_tp_automode_message(self):
        self.tp_automode_message.header.stamp = rospy.get_rostime()
        self.tp_automode_publisher.publish(self.tp_automode_message)


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
            #self.wc.get_kuka_configuration()       # This might slower down publishing automode topics (set timeout in launchfile)
            self.publishing_rate.sleep()

# main function
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('hmi_support_workcell_node')
    try:
        Node()
    except rospy.ROSInterruptException:
        pass


