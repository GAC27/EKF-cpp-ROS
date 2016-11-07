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


class EKF:
    def __init__(self):
        rospy.init_node('EKF', anonymous=True) #make node 
        self.flag_active_odom=False
        self.flag_active_base_scan=False
        self.pub_base_scan = rospy.Publisher("EKF_base_scan", LaserScan, queue_size=0)
        self.pub_odom = rospy.Publisher("EKF_odom", Odometry, queue_size=0)


    def odometryReceived(self,msg):
         self.last_odom = msg
         self.flag_active_odom = True

    def BaseScanReceived(self,msg):
        self.last_base_scan = msg
        self.flag_active_base_scan = True

    def EKFprocess(self):
  
    	print last_odom
    	print "\n\n\n\n\n\n\n"
    	print last_base_scan
    	if(self.flag_active_odom and self.flag_active_base_scan):
    		self.pub_odom.publish(last_odom)
    		self.pub_base_scan.publish(last_base_scan)
    	

    def cleanFlags(self):
    	self.flag_active_odom = False
    	self.flag_active_odom = False



    def EKF_algorithm(self):
        
        
        while not rospy.is_shutdown():
        	rospy.Subscriber('base_scan',LaserScan,BaseScanReceived)
        	rospy.Subscriber('odom',Odometry,odometryReceived)

        	self.EKFprocess()
        	self.cleanFlags()

if __name__ == '__main__':
    try:
        ekf_module = EKF()
        ekf_module.EKF_algorithm()
    except rospy.ROSInterruptException:
        pass