# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Rein Appeldoorn
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

"""Defines the main method for the nmea_socket_driver executable."""


import select
import sys
import traceback
import socket

import rospy

from libnmea_navsat_driver.driver import RosNMEADriver


def nmea_handler(driver, sensor_data):

    sensor_data = sensor_data.decode("utf-8").strip()
    # print("start")
    # print(sensor_data)
    # print("end")

    try:
        nmea_sentences = sensor_data.replace("\r\n", "").split("$")
        # print("start")
        # print(nmea_sentences)
        # print("end")

        rospy.logdebug("start")
        for nmea_sentence in nmea_sentences:
            if not nmea_sentence:
                continue
            nmea_sentence = "$" + nmea_sentence
            rospy.logdebug("nmea:" + nmea_sentence)
            driver.add_sentence(nmea_sentence, RosNMEADriver.get_frame_id()) 
        rospy.logdebug("end")
            
    except UnicodeError as e:
        rospy.logwarn("Skipped reading a line from the UDP socket because it could not be "
                        "decoded as an ASCII string. The bytes were {0}".format(nmea_sentence))
    except ValueError:
        rospy.logwarn(
            "ValueError, likely due to missing fields in the NMEA "
            "message. Please report this issue at "
            "https://github.com/ros-drivers/nmea_navsat_driver"
            ", including the following:\n\n"
            "```\n" +
            repr(nmea_sentence) + "\n\n" +
            traceback.format_exc() +
            "```")


def main():
    rospy.init_node('nmea_tcp_driver', anonymous=True, log_level=rospy.DEBUG)

    try:
        local_ip = rospy.get_param('~ip', '0.0.0.0')
        local_port = rospy.get_param('~port', 10110)
        timeout = rospy.get_param('~timeout_sec', 2)
    except KeyError as e:
        rospy.logerr("Parameter %s not found" % e)
        sys.exit(1)

    # Create a socket
    nmea_driver = RosNMEADriver()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    serverAddress = (local_ip, local_port)
    sock.connect(serverAddress)

    datas = b''
    try:
        while not rospy.is_shutdown():
            data = sock.recv(2048)
            datas += data
            if not data: break
            if data == b"$":
                nmea_handler(driver=nmea_driver, sensor_data=datas)
                datas = b''

    except Exception:
        rospy.logerr(traceback.format_exc())
    finally:
        sock.close()
