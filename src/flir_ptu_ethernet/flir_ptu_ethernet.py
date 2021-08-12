#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math
import urllib
import urllib2
import json

# Insert here msg and srv imports:
from std_msgs.msg import Float64
from robotnik_msgs.msg import PantiltStatus, PantiltStatusStamped
from sensor_msgs.msg import JointState

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
            '~max_pan_speed', 25.0) # deg/s
        self.max_tilt_speed = rospy.get_param(
            '~max_tilt_speed', 25.0) # deg/s
        self.ptu_model = rospy.get_param(
            '~ptu_model', "PTU-D48E") # PTU-5, PTU-D48E

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.pan_pos_sub = rospy.Subscriber(
            'joint_pan_position_controller/command', Float64, self.pan_pos_sub_cb)

        self.tilt_pos_sub = rospy.Subscriber(
            'joint_tilt_position_controller/command', Float64, self.tilt_pos_sub_cb)

        self.pan_speed_sub = rospy.Subscriber(
            'joint_pan_speed_controller/command', Float64, self.pan_speed_sub_cb)

        self.tilt_speed_sub = rospy.Subscriber(
            'joint_tilt_speed_controller/command', Float64, self.tilt_speed_sub_cb)

        self.status_pub = rospy.Publisher(
            '~status', PantiltStatus, queue_size=10)
        self.status_stamped_pub = rospy.Publisher(
            '~status_stamped', PantiltStatusStamped, queue_size=10)
        self.joint_state_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=10)

        return 0

    def init_state(self):
        self.pan_pos = 0 # deg
        self.tilt_pos = 0 # deg
        self.pan_speed = 0 # deg
        self.tilt_speed = 0 # deg

        self.status = PantiltStatus()
        self.status.pan_pos = self.pan_pos
        self.status.tilt_pos = self.tilt_pos

        self.max_pan_pos = 168.00 # deg
        self.min_pan_pos = -167.99 # deg
        self.max_tilt_pos = 30.00 # deg
        self.min_tilt_pos = -89.99 # deg

        # Pantilt encoders resolution (deg/pos)
        if (self.ptu_model == "PTU-5"):
            self.pan_resolution = 0.05
            self.tilt_resolution = 0.05
        elif (self.ptu_model == "PTU-D48E"):
            self.pan_resolution = 0.025714
            self.tilt_resolution = 0.012857
        else:
            rospy.logerr('%s:init_state: %s is not a valid model' % (rospy.get_name(), self.ptu_model))
            self.switch_to_state(State.FAILURE_STATE)

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Publish topic with status

        status_stamped = PantiltStatusStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.pantilt.pan_pos = self.status.pan_pos
        status_stamped.pantilt.tilt_pos = self.status.tilt_pos
        status_stamped.pantilt.pan_speed = self.status.pan_speed
        status_stamped.pantilt.tilt_speed = self.status.tilt_speed

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        # Publish joint_state

        if (self.update_position() == -1):
            self.switch_to_state(State.EMERGENCY_STATE)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name.append("robot_flir_ptu_5_pan_joint")
        joint_state_msg.name.append("robot_flir_ptu_5_tilt_joint")
        joint_state_msg.position.append(self.pan_pos*math.pi/180.0)
        joint_state_msg.position.append(self.tilt_pos*math.pi/180.0)
        joint_state_msg.velocity.append(self.pan_speed*math.pi/180.0)
        joint_state_msg.velocity.append(self.tilt_speed*math.pi/180.0)

        self.joint_state_pub.publish(joint_state_msg)

        return RComponent.ready_state(self)

    def emergency_state(self):
        """Actions performed in emergency state"""

        if (self.update_position() == -1):
            self.switch_to_state(State.EMERGENCY_STATE)
        else:
            self.switch_to_state(State.READY_STATE)

        return RComponent.emergency_state(self)

    def failure_state(self):
        """Actions performed in failure state"""
        self.switch_to_state(State.SHUTDOWN_STATE)

    def send_ptu_command(self, pan_pos, tilt_pos, pan_speed, tilt_speed):
        params = urllib.urlencode({'PP': pan_pos/self.pan_resolution, 'TP': tilt_pos/self.tilt_resolution,'PS': pan_speed/self.pan_resolution, 'TS': tilt_speed/self.tilt_resolution, 'C': 'I'})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError as e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError as e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_pan_pos_command(self, pan_pos):
        pan_pos = self.clamp(pan_pos, self.min_pan_pos, self.max_pan_pos)
        params = urllib.urlencode({'PP': pan_pos/self.pan_resolution, 'PS': self.max_pan_speed/self.pan_resolution, 'C': 'I'})
        for _ in range(2):
            try:
                ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
            except IOError as e:
                rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
                return -1
            except ValueError as e:
                rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_tilt_pos_command(self, tilt_pos):
        tilt_pos = self.clamp(tilt_pos, self.min_tilt_pos, self.max_tilt_pos)
        params = urllib.urlencode({'TP': tilt_pos/self.tilt_resolution, 'TS': self.max_tilt_speed/self.tilt_resolution, 'C': 'I'})
        for _ in range(2):
            try:
                ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
            except IOError as e:
                rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
                return -1
            except ValueError as e:
                rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_pan_speed_command(self, pan_speed):
        pan_speed = self.clamp(pan_speed, -self.max_pan_speed, self.max_pan_speed)
        params = urllib.urlencode({'PS': pan_speed/self.pan_resolution, 'C': 'V'})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError as e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError as e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def send_tilt_speed_command(self, tilt_speed):
        tilt_speed = self.clamp(tilt_speed, -self.max_tilt_speed, self.max_tilt_speed)
        params = urllib.urlencode({'TS': tilt_speed/self.tilt_resolution, 'C': 'V'})
        try:
            ptu_post = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=params, timeout=2)
        except IOError as e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError as e:
            rospy.logwarn('%s:update_position: %s' % (rospy.get_name(), e))
        return 0

    def update_position(self):
        pan_pos_param = urllib.urlencode({'PP': ''})
        tilt_pos_param = urllib.urlencode({'TP': ''})
        pan_speed_param = urllib.urlencode({'PD': ''})
        tilt_speed_param = urllib.urlencode({'TD': ''})
        try:
            pan_pos = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=pan_pos_param, timeout=2)
            self.pan_pos = int(json.load(pan_pos)["PP"])*self.pan_resolution
            tilt_pos = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=tilt_pos_param, timeout=2)
            self.tilt_pos = int(json.load(tilt_pos)["TP"])*self.tilt_resolution
            pan_speed = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=pan_speed_param, timeout=2)
            self.pan_speed = int(json.load(pan_speed)["PD"])*self.pan_resolution
            tilt_speed = urllib2.urlopen("http://"+self.ip+"/API/PTCmd", data=tilt_speed_param, timeout=2)
            self.tilt_speed = int(json.load(tilt_speed)["TD"])/100.0
            self.status.pan_pos = self.pan_pos
            self.status.tilt_pos = self.tilt_pos
            self.status.pan_speed = self.pan_speed
            self.status.tilt_speed = self.tilt_speed
        except IOError, e:
            rospy.logwarn('%s:update_position: %s %s' % (rospy.get_name(), e, self.ip))
            return -1
        except ValueError as e:
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

    def pan_speed_sub_cb(self, msg):
        self.send_pan_speed_command(msg.data*180/math.pi)

    def tilt_speed_sub_cb(self, msg):
        self.send_tilt_speed_command(msg.data*180/math.pi)
