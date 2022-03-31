#!/usr/bin/env python3

# Copyright (c) 2022, maxon motor ag
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the University of California, Berkeley nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Copyright 2022 by maxon motor ag
# All rights reserved.
# @file python_example_1dof_ppm.py
# @author Cyril Jourdan
# @date Mar 22, 2022
# @brief Example code to control one EPOS4 with ROS2 ros1_bridge
# This program sends a discrete sinusoidal wave of target positions, to be used with 1 DOF in PPM 
# Contact: cyril.jourdan@roboprotos.com

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 
import math


class EPOS4CmdPublisher(Node):

    def __init__(self):
        super().__init__('epos4_cmd_publisher')
        self.publisher_ = self.create_publisher(Float64, '/maxon/canopen_motor/base_link1_joint_position_controller/command', 1) # define a publisher
        self.timer_period = 0.2  # interval between two consecutive motor commands, in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0 # steps
        self.period_sec = 60.0 # wave period in seconds
        self.amplitude = 100000 # wave amplitude in inc 

    def timer_callback(self):
        msg = Float64()
        msg.data = float(round(self.amplitude*math.sin((self.i*self.timer_period)*2*math.pi/self.period_sec))) # calculate a new target position 
        self.publisher_.publish(msg) # publish the new target position to the command topic
        self.get_logger().info('pos: "%d" inc' % msg.data)
        self.i += 1 # increment step


def main(args=None):
    rclpy.init(args=args)

    epos4_cmd_publisher = EPOS4CmdPublisher()

    rclpy.spin(epos4_cmd_publisher)

    # Destroy the node
    epos4_cmd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
