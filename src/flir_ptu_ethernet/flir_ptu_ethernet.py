#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math
import urllib

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse


class FlirPtuEthernet(RComponent):
    """
    Package for controlling the FLIR PTU using ROS.
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.example_subscriber_name = rospy.get_param(
            '~example_subscriber_name', 'example')

        self.ip = rospy.get_param(
            '~ip_address', '192.168.0.180')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.example_sub = rospy.Subscriber(
            self.example_subscriber_name, String, self.example_sub_cb)

        self.data_pub = rospy.Publisher(
            '~data', String, queue_size=10)
        self.data_stamped_pub = rospy.Publisher(
            '~data_stamped', StringStamped, queue_size=10)

        self.example_server = rospy.Service(
            '~example', Trigger, self.example_server_cb)

        return 0

    def init_state(self):
        self.data = String()

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Publish topic with data

        data_stamped = StringStamped()
        data_stamped.header.stamp = rospy.Time.now()
        data_stamped.string = self.data.data

        self.data_pub.publish(self.data)
        self.data_stamped_pub.publish(data_stamped)

        # Publish command

        self.pan_pos = 0 # -16799 to 16800
        self.tilt_pos = 0 # -8999 to 3000
        self.max_pan_speed = 12000 # -12000 to 12000
        self.max_tilt_speed = 12000 # -12000 to 12000
        self.send_ptu_command(self.pan_pos, self.tilt_pos, self.max_pan_speed, self.max_tilt_speed)

        return RComponent.ready_state(self)


    def send_ptu_command(self, pan_pos, tilt_pos, pan_speed, tilt_speed):
        params = urllib.urlencode({'PP': pan_pos, 'TP': tilt_pos, 'PS': pan_speed, 'TS': tilt_speed})
        try:
            f = urllib.urlopen("http://"+self.ip+"/API/PTCmd", params)
        except IOError, e:
            rospy.logerr('%s:send_ptu_command: %s %s' % (rospy.get_name(), e, self.ip))
        #print f.read()

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def example_sub_cb(self, msg):
        rospy.logwarn("Received msg: " + msg.data)

    def example_server_cb(self, req):
        rospy.logwarn("Received srv trigger petition.")

        response = TriggerResponse()
        response.success = True
        response.message = "Received srv trigger petition."
        return response
