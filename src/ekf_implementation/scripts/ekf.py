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
current_odom = 0
current_base_scan = 0

class Orientation:
    def __init__(self,x,y,z,w):
        self.x=x
        self.y=y
        self.z=z
        self.w=w

class Position:
    def __init__(self,x,y,z):
        self.x=x
        self.y=y
        self.z=z

class State:
    def __init__(self, x=0, y=0, z=0, orientationX=0,orientationY=0,orientationZ=0,orientationW=0):
        self.position=Position(x,y,z)
        self.orientation=Orientation(orientationX,orientationY,orientationZ,orientationW)





def odometryReceived(msg):
    global current_odom 
    global last_odom 
    last_odom = current_odom
    current_odom = msg
    if(last_odom == 0):
        last_odom = current_odom
    global flag_active_odom 
    flag_active_odom = True

def BaseScanReceived(msg):
    global current_base_scan 
    current_base_scan = msg
    global flag_active_base_scan
    flag_active_base_scan = True

def EKFprocess(currentState):
    global flag_active_odom
    global flag_active_base_scan
    global last_odom
    global current_odom
    global current_base_scan

    predictedState=State()
    predictedObservation=0

    if(flag_active_odom and flag_active_base_scan):
        pub_odom.publish(current_odom)
        pub_base_scan.publish(current_base_scan)
        predictedState=getPredictedState(currentState,getPoseDifference(last_odom.pose.pose,current_odom.pose.pose))
        #Prediction(predictedState, covariance)
        return predictedState
    else:
        return currentState



    

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
    currentState=State()

    while not rospy.is_shutdown():
        rospy.Subscriber('base_scan',LaserScan,BaseScanReceived)
        rospy.Subscriber('odom',Odometry,odometryReceived)
        currentState=EKFprocess(currentState)
        cleanFlags()

#Receives 2 geometry_msg/Pose Messages
#Returns a List of floats with position
#   U(X)
def getPoseDifference(lastPose,currentPose):
    deltaPose=[]

    #Position
    deltaPose.append(currentPose.position.x - lastPose.position.x)
    deltaPose.append(currentPose.position.y - lastPose.position.y)
    deltaPose.append(currentPose.position.z - lastPose.position.z)

    #Orientation
    deltaPose.append(currentPose.orientation.x - lastPose.orientation.x)
    deltaPose.append(currentPose.orientation.y - lastPose.orientation.y)
    deltaPose.append(currentPose.orientation.z - lastPose.orientation.z)
    deltaPose.append(currentPose.orientation.w - lastPose.orientation.w)

    return deltaPose


#Returns a new state ~Xk+1 given an action Uk
def F(Xk,Uk):
    posX=Xk.position.x + Uk[0]
    posY=Xk.position.y + Uk[1]
    posZ=Xk.position.z + Uk[2]
    
    orientationX=Xk.orientation.x + Uk[3]
    orientationY=Xk.orientation.y + Uk[4]
    orientationZ=Xk.orientation.z + Uk[5]
    orientationW=Xk.orientation.w + Uk[6]

    return State(posX,posY,posZ,orientationX,orientationY,orientationZ,orientationW)

#Returns the error
def V(Xk):
    return 0

#Returns a new state ~Xk+1 given an action Uk and applies noise to it
def getPredictedState(Xk,Uk):
    newState=F(Xk,Uk)
    noise=V(Xk)
    newState.position.x = newState.position.x + noise
    newState.position.y = newState.position.y + noise
    newState.position.z = newState.position.z + noise

    newState.orientation.x = newState.orientation.x + noise
    newState.orientation.y = newState.orientation.y + noise
    newState.orientation.z = newState.orientation.z + noise
    newState.orientation.w = newState.orientation.w + noise
    return newState


#TODO
#def Prediction(state,covariance):
    #newState = 
    #TODO

if __name__ == '__main__':
    try:
        EKF()
    except rospy.ROSInterruptException:
        pass