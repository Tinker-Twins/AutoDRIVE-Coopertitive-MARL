#!/usr/bin/env python3

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
from ackermann_msgs.msg import AckermannDriveStamped

################################################################################

DRIVE_LIMIT = 1.0 # m/s
STEER_LIMIT = 0.52 # rad

pub_cosim_command = rospy.Publisher('/autodrive/f1tenth_1/cosim_mode', Int32, queue_size=1)
pub_pose_command = rospy.Publisher('/autodrive/f1tenth_1/pose_command', Pose, queue_size=1)

pub_ackermann_command = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

global ackermann_command
ackermann_command = AckermannDriveStamped()

def callback_cosim(cosim_data):
    global ackermann_command
    cosim_mode = Int32()
    pose_command = Pose()
    cosim_mode = 1
    pose_command.position.x = cosim_data.pose.pose.position.x + 14.8
    pose_command.position.y = cosim_data.pose.pose.position.y - 6.84
    pose_command.position.z = cosim_data.pose.pose.position.z
    pose_command.orientation.x = cosim_data.pose.pose.orientation.x
    pose_command.orientation.y = cosim_data.pose.pose.orientation.y
    pose_command.orientation.z = cosim_data.pose.pose.orientation.z - 0.7071068
    pose_command.orientation.w = cosim_data.pose.pose.orientation.w + 0.7071068
    ackermann_command.header.stamp = rospy.Time.now()
    ackermann_command.header.frame_id = 'base_link'
    pub_cosim_command.publish(cosim_mode)
    pub_pose_command.publish(pose_command)
    pub_ackermann_command.publish(ackermann_command)

def callback_throttle(throttle_data):
    global ackermann_command
    ackermann_command.drive.speed = throttle_data.data * DRIVE_LIMIT

def callback_steering(steering_data):
    global ackermann_command
    ackermann_command.drive.steering_angle = steering_data.data * STEER_LIMIT

################################################################################

if __name__=="__main__":

    rospy.init_node('cosim')
    rospy.Subscriber('/vesc/odom', Odometry, callback_cosim)
    rospy.Subscriber('/autodrive/f1tenth_1/throttle', Float32, callback_throttle)
    rospy.Subscriber('/autodrive/f1tenth_1/steering', Float32, callback_steering)
    rospy.spin()