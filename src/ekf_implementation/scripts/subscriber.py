#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

flag_active_odom = 0
flag_active_base_scan = 0
last_odom = 0
last_base_scan = 0


def odometryReceived(msg):
     global last_odom 
     last_odom = msg
     global flag_active_odom 
     flag_active_odom = True

def BaseScanReceived(msg):
    global last_base_scan 
    last_base_scan = msg
    global flag_active_base_scan
    flag_active_base_scan = True

def EKFprocess():
	global flag_active_odom
	global flag_active_base_scan
	global last_odom
	global last_base_scan
	print last_odom
	print "\n\n\n\n\n\n\n"
	print last_base_scan
	if(flag_active_odom and flag_active_base_scan):
		pub_odom.publish(last_odom)
		pub_base_scan.publish(last_base_scan)
	

def cleanFlags():
	global flag_active_odom 
	flag_active_odom = False
	global flag_active_base_scan 
	flag_active_odom = False



def EKF():
    rospy.init_node('EKF', anonymous=True) #make node 
    #rospy.Subscriber('odom',Odometry,odometryReceived)
    global pub_base_scan
    pub_base_scan = rospy.Publisher("EKF_base_scan", LaserScan, queue_size=0)
    global pub_odom
    pub_odom = rospy.Publisher("EKF_odom", Odometry, queue_size=0)
    global flag_active_odom
    flag_active_odom = False
    global flag_active_base_scan
    flag_active_base_scan = False
    while not rospy.is_shutdown():
    	rospy.Subscriber('base_scan',LaserScan,BaseScanReceived)
    	rospy.Subscriber('odom',Odometry,odometryReceived)

    	EKFprocess()
    	cleanFlags()

if __name__ == '__main__':
    try:
        EKF()
    except rospy.ROSInterruptException:
        pass