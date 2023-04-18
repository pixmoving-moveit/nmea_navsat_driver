#! /usr/bin/env python
import select
import sys
import traceback
import socket

import rospy

from libnmea_navsat_driver.driver import RosNMEADriver

def main():
    rospy.init_node('nmea_socket_driver')

    try:
        local_ip = rospy.get_param('~ip', '192.168.1.110')
        local_port = rospy.get_param('~port', 9904)
        timeout = rospy.get_param('~timeout_sec', 2)
        buffer_size = rospy.get_param('~buffer_size', 4096)
    except KeyError as e:
        rospy.logerr("Parameter %s not found" % e)
        sys.exit(1)
    
    driver = RosNMEADriver()
    frame_id = driver.get_frame_id()

    while not rospy.is_shutdown():
        try:
            SERVER_ADDRESS = (local_ip, local_port)
            gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            gnss_socket.connect(SERVER_ADDRESS)
        except socket.error as exc:
            rospy.logerr("Caught exception socket.error when setting up socket: %s" % exc)
            sys.exit(1)
        
        partial = ""
        while not rospy.is_shutdown():
            try:
                partial += gnss_socket.recv(buffer_size)

                # strip the data
                lines = partial.splitlines()
                if partial.endswith('\n'):
                    full_lines = lines
                    partial = ""
                else:
                    full_lines = lines[:-1]
                    partial = lines[-1]

                for data in full_lines:
                    try:
                        if driver.add_sentence(data, frame_id):
                            rospy.loginfo_throttle(5,"Received sentence: %s" % data)
                        else:
                            rospy.logwarn_throttle(5,"Error with sentence: %s" % data)
                    except ValueError as e:
                        rospy.logwarn(
                            "Value error, likely due to missing fields in the NMEA message. "
                            "Error was: %s. Please report this issue to me. " % e)

            except socket.error as exc:
                rospy.logerr("Caught exception socket.error when receiving: %s" % exc)
                gnss_socket.close()
                break

        gnss_socket.close()
        
if __name__=="__main__":
    main()