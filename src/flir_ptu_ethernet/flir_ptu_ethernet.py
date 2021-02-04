#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math
import urllib
import urllib2
import json

# Insert here msg and srv imports:
from std_msgs.msg import Float64, String
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

        self.ip = rospy.get_param(
            '~ip_address', '192.168.0.180')
        self.max_pan_speed = rospy.get_param(
            '~max_pan_speed', 120.0) # deg/s
        self.max_tilt_speed = rospy.get_param(
            '~max_tilt_speed', 120.0) # deg/s

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.pan_pos_sub = rospy.Subscriber(
            'joint_pan_position_controller/command', Float64, self.pan_pos_sub_cb)

        self.tilt_pos_sub = rospy.Subscriber(
            'joint_tilt_position_controller/command', Float64, self.tilt_pos_sub_cb)

        self.data_pub = rospy.Publisher(
            '~data', String, queue_size=10)
        self.data_stamped_pub = rospy.Publisher(
            '~data_stamped', StringStamped, queue_size=10)

        self.example_server = rospy.Service(
            '~example', Trigger, self.example_server_cb)

        return 0

    def init_state(self):
        self.pan_pos = 0 # deg
        self.tilt_pos = 0 # deg

        self.data = String()
        self.data.data = "Pan: "+str(self.pan_pos)+", Tilt: "+str(self.tilt_pos)+" (deg)"

        self.max_pan_pos = 168.00 # deg
        self.min_pan_pos = -167.99 # deg
        self.max_tilt_pos = 30.00 # deg
        self.min_tilt_pos = -89.99 # deg


        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Publish topic with data

        data_stamped = StringStamped()
        data_stamped.header.stamp = rospy.Time.now()
        data_stamped.string = self.data.data

        self.data_pub.publish(self.data)
        self.data_stamped_pub.publish(data_stamped)

        if (self.update_position() == -1):
            self.switch_to_state(State.EMERGENCY_STATE)

        return RComponent.ready_state(self)

    def emergency_state(self):
        """Actions performed in emergency state"""

        if (self.update_position() == -1):
            self.switch_to_state(State.EMERGENCY_STATE)
        else:
            self.switch_to_state(State.READY_STATE)

        return RComponent.emergency_state(self)


    def send_ptu_command(self, pan_pos, tilt_pos, pan_speed, tilt_speed):
        params = urllib.urlencode({'PP': pan_pos*100, 'TP': tilt_pos*100, 'PS': pan_speed*100, 'TS': tilt_speed*100})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError, e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError, e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_pan_pos_command(self, pan_pos):
        pan_pos = self.clamp(pan_pos, self.min_pan_pos, self.max_pan_pos)
        params = urllib.urlencode({'PP': pan_pos*100, 'PS': self.max_pan_speed*100})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError, e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError, e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_tilt_pos_command(self, tilt_pos):
        tilt_pos = self.clamp(tilt_pos, self.min_tilt_pos, self.max_tilt_pos)
        params = urllib.urlencode({'TP': tilt_pos*100, 'TS': self.max_tilt_speed*100})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError, e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError, e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_pan_speed_command(self, pan_speed):
        pan_speed = self.clamp(pan_speed, -self.max_pan_speed, self.max_pan_speed)
        params = urllib.urlencode({'PS': pan_speed*100})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError, e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError, e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_tilt_speed_command(self, tilt_speed):
        tilt_speed = self.clamp(tilt_speed, -self.max_tilt_speed, self.max_tilt_speed)
        params = urllib.urlencode({'TS': tilt_speed*100})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError, e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError, e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def update_position(self):
        pan_param = urllib.urlencode({'PP': ''})
        tilt_param = urllib.urlencode({'TP': ''})
        try:
            pan_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=pan_param, timeout=2)
            self.pan_pos = int(json.load(pan_post)["PP"])/100.0
            tilt_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=tilt_param, timeout=2)
            self.tilt_pos = int(json.load(tilt_post)["TP"])/100.0
            self.data.data = "Pan: "+str(self.pan_pos)+", Tilt: "+str(self.tilt_pos)+" (deg)"
        except IOError, e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError, e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0


    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)


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

    def pan_pos_sub_cb(self, msg):
        self.send_pan_pos_command(msg.data*180/math.pi)

    def tilt_pos_sub_cb(self, msg):
        self.send_tilt_pos_command(msg.data*180/math.pi)

    def example_server_cb(self, req):
        rospy.logwarn("Received srv trigger petition.")

        response = TriggerResponse()
        response.success = True
        response.message = "Received srv trigger petition."
        return response
