#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flir_ptu_ethernet import FlirPtuEthernet


def main():

    rospy.init_node("flir_ptu_ethernet_node")

    rc_node = FlirPtuEthernet()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
