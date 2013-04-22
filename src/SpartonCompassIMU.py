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
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

import serial, math, time, re, select

# forget ms_delay
SCRIPT = """
variable ms_delay
: position_northtek
pitch di@ f. ." ," 
roll di@ f. ." ," 
yaw di@ f. ." ,"
accelp di@ @ f. ." ,"
accelp di@ 4 + @ f. ." ,"
accelp di@ 8 + @ f. ." ,"
gyrop di@ @ f. ." ,"
gyrop di@ 4 + @ f. ." ,"
gyrop di@ 8 + @ f. ." ,"
quaternion di@ @ f. ." ,"
quaternion di@ 4 + @ f. ." ,"
quaternion di@ 8 + @ f. ." ,"
quaternion di@ 12 + @ f. ." \\r\\n"
;
: tOutput 
100 ms_delay !
time
begin 
?key 0= while
begin 
time over - ms_delay @ < while
repeat drop time                   
position_northtek
repeat 
drop
;
"""

def _shutdown():
    global ser
    rospy.loginfo("Sparton shutting down.")
    ser.write("\x1a")
    rospy.loginfo('Closing Digital Compass Serial port')
    ser.close()

def serial_lines(ser, brk="\r\n"):
    buf = ""
    while True:
        rlist, _, _ = select.select([ ser ], [], [], 1.0)
        if not rlist:
            continue
        new = ser.read(ser.inWaiting())
        buf += new
        while brk in buf:
            msg, buf = buf.split(brk)[-2:]
            yield msg

if __name__ == '__main__':
    global ser
    rospy.init_node('SpartonDigitalCompassIMU')
    rpy_pub = rospy.Publisher('imu/rpy', Vector3Stamped)
    imu_pub = rospy.Publisher('imu/data', Imu)

    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = rospy.get_param('~baud', 115200)
    compass_offset_degrees = rospy.get_param('~offset', 0.0)

    rpy_data = Vector3Stamped(header=rospy.Header(frame_id="imu"))
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
        rospy.loginfo("Opening serial port %s for digital compass." % port)
        ser = serial.Serial(port=port, baudrate=baud, timeout=.5)
        ser.write("\x1a")
        time.sleep(0.05) 
        ser.flushInput()
        lines = serial_lines(ser)

        # Send output script. TODO: Verify this after programming.
        rospy.loginfo("Sending output program to digital compass.")
        success = False
        while not success:
            success = True
            for output_line in SCRIPT.splitlines():
                # rospy.loginfo("TX: %s" % output_line) 
                ser.write("%s\r\n" % output_line)
                time.sleep(0.05) 
                input_line = lines.next()
                if input_line == output_line:
                    # Line echoed back exactly.
                    continue
                if input_line == output_line + "OK":
                    # Line echoed with acknowledgment.
                    continue
                if input_line.startswith("forget") and (output_line.startswith("Can't find") or output_line == "OK"):
                    # Special case for the variable forget on startup.
                    continue

                print repr(input_line), repr(output_line)
                rospy.logwarn("Bad output program. Retrying now.")
                success = False
                break
                    
        rospy.loginfo("Program sent successfully. Commencing sensor output.")
                    
        time.sleep(0.2)
        ser.flushInput()
        
        while True:
            ser.write("tOutput\r")
            input_line = lines.next()
            if input_line.startswith('tOutput'):
                rospy.loginfo("Sensor output beginning.")
                break 

        while not rospy.is_shutdown(): 
            data = lines.next()
            #rospy.loginfo("RX: %s" % data) 
            
            try:
                fields = map(float, data.split(","))
                pitch, roll, yaw, ax, ay, az, gx, gy, gz, qx, qy, qz, qw = fields        

                imu_data.header.stamp = rospy.Time.now()
                imu_data.orientation.x = qy
                imu_data.orientation.y = qx
                imu_data.orientation.z = -qz
                imu_data.orientation.w = qw
                imu_data.angular_velocity.x = gy
                imu_data.angular_velocity.y = gx
                imu_data.angular_velocity.z = -gz
                imu_data.linear_acceleration.x = ay
                imu_data.linear_acceleration.y = ax
                imu_data.linear_acceleration.z = -az
                imu_pub.publish(imu_data)

                rpy_data.header.stamp = rospy.Time.now()
                rpy_data.vector.x = math.radians(roll)
                rpy_data.vector.y = math.radians(pitch)
                rpy_data.vector.z = math.radians(yaw)
                rpy_pub.publish(rpy_data)

            except ValueError as e:
                rospy.logerr(str(e))
                continue

        rospy.loginfo('Closing Digital Compass Serial port')
        ser.close()
    except rospy.ROSInterruptException:
        pass
