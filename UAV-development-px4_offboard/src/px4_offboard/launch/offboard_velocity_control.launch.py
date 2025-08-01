#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import cv2
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        #Node(
            #package='optical_flow',
            #namespace='optical_flow',
            #executable='optical_flow_node',
            #name='optical_flow_node',
            #output='screen',
            #prefix='screen -dmS optical_flow_node'
            #prefix='gnome-terminal --geometry=65x11 --'
        #),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer',
            #output='screen',
            #prefix='screen -dmS visualizer'
            #prefix='gnome-terminal --geometry=65x11 --'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            #output='screen',
            #prefix='screen -dmS processes'
            #prefix='gnome-terminal --geometry=110x20 --'
        ),
        #Node(
            #package='px4_offboard',
            #namespace='px4_offboard',
            #executable='control',
            #name='control',
            #output='screen',
            #prefix='screen -dmS control'
            #prefix='gnome-terminal --geometry=65x24 --',
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity',
            #output='screen',
            #prefix='screen -dmS velocity_control'
        ),
        #Node(
            #package='px4_offboard',
            #namespace='px4_offboard',
            #executable='height_baro',
            #name='height_baro',
            #output='screen',
            #prefix='screen -dmS velocity_control'
        #),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='baro_to_altitude',
            name='raw_baro_node',
            #prefix='gnome-terminal --geometry=65x11 --',
        ),
        
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='sensor_splitter',
            name='sensor_splitter',
            #prefix='gnome-terminal --geometry=65x11 --',
        ),

        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='imu_raw_publisher',
            name='raw_imu',
        ),
        #Node(
            #package='px4_offboard',
            #namespace='px4_offboard',
            #executable='accel_raw_publisher',
            #name='raw_accel',
        #),
        #Node(
            #package='px4_offboard',
            #namespace='px4_offboard',
            #executable='gyro_raw_publisher',
            #name='raw_gyro',
        #),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='mag_raw_publisher',
            name='raw_mag',
        ),

#        Node(
#            package='rviz2',
 #           namespace='',
  #          executable='rviz2',
   #         name='rviz2',
    #        arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
     #   )
    ])

# --geometry=740x450+1180+90
# --geometry=740x540+1180+540
