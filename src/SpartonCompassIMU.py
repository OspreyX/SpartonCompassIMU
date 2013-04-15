#!/usr/bin/env python
# Software License Agreement (BSD License)

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.
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
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
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

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.

# Changelog

# 2013.04.15 Switch to Vector3 message for reporting roll/pitch/yaw.
# 2013.01.06 Add IMU message
# 2012.12.13 Use Pos2D message, normalized to 0 ~ 2*PI
#


import roslib; roslib.load_manifest('SpartonCompassIMU')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

import serial, math, time, re, select

#import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

START_OUTPUTS = [
#"1 accelp.p\r",
#"1 gyrop.p\r",
"1 quat.p\r"
"1 compass.p\r"
]

STOP_OUTPUTS = [
#"0 accel.p\r",
#"0 gyro.p\r",
"0 quat.p\r"
"0 compass.p\r"
]

def wrapTo2PI(theta):
    '''Normalize an angle in radians to [0, 2*pi]
    '''
    return theta % (2.*math.pi)

def wrapToPI(theta):
    '''Normalize an angle in radians to [-pi, pi]
    '''
    return (wrapTo2PI(theta+math.pi) - math.pi)

def _shutdown():
    global ser
    rospy.loginfo("Sparton shutting down.")
    for output in STOP_OUTPUTS:
        ser.write(output)
    rospy.loginfo('Closing Digital Compass Serial port')
    ser.close()

def serial_lines(ser, brk="\n"):
    buf = ""
    while True:
        rlist, _, _ = select.select([ ser ], [], [], 1.0)
        if not rlist:
            continue
        new = ser.read(ser.inWaiting())
        buf += new
        if brk in new:
            msg, buf = buf.split(brk)[-2:]
            yield msg

if __name__ == '__main__':
    global ser
    rospy.init_node('SpartonDigitalCompassIMU')
    rpy_pub = rospy.Publisher('imu/rpy', Vector3)
    imu_pub = rospy.Publisher('imu/data', Imu)

    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = rospy.get_param('~baud', 115200)
    compass_offset_degrees = rospy.get_param('~offset', 0.0)

    rpy_data = Vector3()
    imu_data = Imu(header=rospy.Header(frame_id="imu"))
    
    #TODO find a right way to convert imu acceleration/angularvel./orientation accuracy to covariance
    imu_data.orientation_covariance = [1e-6, 0, 0, 
                                       0, 1e-6, 0, 
                                       0, 0, 1e-6]
    
    imu_data.angular_velocity_covariance = [1e-6, 0, 0,
                                            0, 1e-6, 0, 
                                            0, 0, 1e-6]
    
    imu_data.linear_acceleration_covariance = [1e-6, 0, 0, 
                                               0, 1e-6, 0, 
                                               0, 0, 1e-6]
    rospy.on_shutdown(_shutdown)

    try:
        #Setup Compass serial port
        ser = serial.Serial(port=port, baudrate=baud, timeout=.5)

        for output in START_OUTPUTS:
            rospy.loginfo("TX: %s" % output) 
            ser.write(output)

        lines = serial_lines(ser)

        while not rospy.is_shutdown(): 
            data = lines.next()
            #rospy.loginfo("RX: %s" % data) 
            
            try:
                msg, contents = re.split("[:,]", data, 1)

                fields = map(float, contents.split(","))
                #print msg, fields
        
                if msg == "QUAT":
                    w, x, y, z = fields
                    imu_data.orientation.x = y
                    imu_data.orientation.y = x
                    imu_data.orientation.z = -z
                    imu_data.orientation.w = w
                    imu_pub.publish(imu_data)
                elif msg == "C":
                    timestamp, roll, pitch, yaw = fields
                    rpy_data.x = math.radians(roll)
                    rpy_data.y = math.radians(pitch)
                    rpy_data.z = math.radians(yaw)
                    rpy_pub.publish(rpy_data)
            except ValueError as e:
                rospy.logerr(str(e))
                continue

        rospy.loginfo('Closing Digital Compass Serial port')
        ser.close() #Close D_Compass serial port
    except rospy.ROSInterruptException:
        pass
