#!/usr/bin/env python

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

import rospy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

################################################################################

pub_cosim_command = rospy.Publisher('/autodrive/nigel_1/cosim_mode', Int32, queue_size=1)
pub_pose_command = rospy.Publisher('/autodrive/nigel_1/pose_command', Pose, queue_size=1)

pub_throttle_command = rospy.Publisher('/autodrive/nigel_1/throttle_command', Float32, queue_size=1)
pub_steering_command = rospy.Publisher('/autodrive/nigel_1/steering_command', Float32, queue_size=1)

def callback_cosim(cosim_data):
    cosim_mode = Int32()
    pose_command = Pose()
    cosim_mode = 1
    pose_command.position.x = cosim_data.pose.pose.position.x
    pose_command.position.y = cosim_data.pose.pose.position.y
    pose_command.position.z = cosim_data.pose.pose.position.z
    pose_command.orientation.x = cosim_data.pose.pose.orientation.x
    pose_command.orientation.y = cosim_data.pose.pose.orientation.y
    pose_command.orientation.z = cosim_data.pose.pose.orientation.z
    pose_command.orientation.w = cosim_data.pose.pose.orientation.w
    pub_cosim_command.publish(cosim_mode)
    pub_pose_command.publish(pose_command)

def callback_throttle(throttle_data):
    throttle_command = Float32()
    throttle_command.data = throttle_data.data
    pub_throttle_command.publish(throttle_command)

def callback_steering(steering_data):
    steering_command = Float32()
    steering_command.data = steering_data.data
    pub_steering_command.publish(steering_command)

################################################################################

if __name__=="__main__":

    rospy.init_node('cosim')
    rospy.Subscriber('/odom', Odometry, callback_cosim)
    rospy.Subscriber('/autodrive/nigel_1/throttle', Float32, callback_throttle)
    rospy.Subscriber('/autodrive/nigel_1/steering', Float32, callback_steering)
    rospy.spin()