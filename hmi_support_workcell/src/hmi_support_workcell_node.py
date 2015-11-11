#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from msgs.msg import BoolStamped


class Node():

    def __init__(self):
        """ Node initialization """

        ''' Holding variables '''
        self.current_kuka_configuration = None
        self.wc_automode_msg = BoolStamped()
        self.wc_automode_msg.data = False

        ''' Topics names '''
        self.ui_str_control_wc_topic_name = rospy.get_param('~ui_str_control_wc_sub', '/ui_str_control_wc')
        self.wc_automode_topic_name = rospy.get_param('~wc_automode_pub', '/wc_automode')

        ''' Services names and types '''
        self.manual_conf_service_name = rospy.get_param('~manual_conf_service_name', '/rsdPlugin/SetConfiguration')
        self.manual_conf_service_type = rospy.get_param('~manual_conf_service_type', '/kuka_ros/setConfiguration.srv')
        self.get_conf_service_name = rospy.get_param('~get_conf_service_name', '/KukaNode/GetConfiguration')
        self.get_conf_service_type = rospy.get_param('~get_conf_service_type', '/kuka_ros/getConfiguration.srv')

        ''' Other Parameters '''
        self.kuka_joints_steps = rospy.get_param('~kuka_joints_steps', [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.update_rate = rospy.Rate(rospy.get_param('~update_rate', 20))  # Hz

        ''' Publishers '''
        self.wc_automode_publisher = rospy.Publisher(self.wc_automode_topic_name, BoolStamped, queue_size=1)

        ''' Subscribers '''
        rospy.Subscriber(self.ui_str_control_wc_topic_name, String, self.on_ui_str_control_wc_topic)

        ''' Start update loop '''
        self.updater()

    def on_ui_str_control_wc_topic(self, msg):
        if msg.data.startswith('wc_joint'):
            joint_ind = int(msg.data.split('_')[2][1])
            if msg.data.endswith('d'):
                self.current_kuka_configuration[joint_ind] -= self.kuka_joints_steps[joint_ind]
            elif msg.data.endswith('u'):
                self.current_kuka_configuration[joint_ind] += self.kuka_joints_steps[joint_ind]
            self.forward_configuration()
        elif msg.data == 'wc_mode_auto':
            self.wc_automode_msg.data = True
            self.publish_automode_message()     # TODO : publish wc_automode 'all the time' or only when changed?
        elif msg.data == 'wc_mode_manual':
            self.wc_automode_msg.data = False
            self.publish_automode_message()
        else:
            print 'This structure of message is not yet implemented to be processed:', msg.data

    def get_kuka_configuration(self):
        rospy.wait_for_service(self.get_conf_service_name)
        try:
            service = rospy.ServiceProxy(self.get_conf_service_name, self.get_conf_service_type)
            self.current_kuka_configuration = service()     # TODO : will this be ok? -> maybe need to transform to .py list object
        except rospy.ServiceException:
            print 'Service call (get_configuration) failed.'

    def forward_configuration(self):
        rospy.wait_for_service(self.manual_conf_service_name)
        try:
            service = rospy.ServiceProxy(self.manual_conf_service_name, self.manual_conf_service_type)
            service(self.current_kuka_configuration)
        except rospy.ServiceException:
            print 'Service call (manual_configuration) failed.'

    def publish_automode_message(self):
        self.wc_automode_msg.header.stamp = rospy.get_rostime()
        self.wc_automode_publisher.publish(self.wc_automode_msg)

    def updater(self):
        while not rospy.is_shutdown():
            self.get_kuka_configuration()
            self.update_rate.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('hmi_support_workcell_node')
    try:
        Node()
    except rospy.ROSInterruptException:
        pass


