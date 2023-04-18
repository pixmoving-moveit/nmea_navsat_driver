"""Defines the main method for the nmea_tcp_driver executable."""

import select
import sys
import traceback

try:
    import socketserver
except ImportError:
    import SocketServer as socketserver  # Python 2.7

import rospy

from libnmea_navsat_driver.driver import RosNMEADriver


class NMEAMessageHandler(socketserver.StreamRequestHandler):
    def handle(self):
        while not rospy.is_shutdown():
            line = self.rfile.readline().strip()
            if not line:
                continue

            try:
                self.server.driver.add_sentence(line, self.server.frame_id)
            except ValueError:
                rospy.logwarn(
                    "ValueError, likely due to missing fields in the NMEA "
                    "message. Please report this issue at "
                    "https://github.com/ros-drivers/nmea_navsat_driver"
                    ", including the following:\n\n"
                    "```\n" +
                    repr(line) + "\n\n" +
                    traceback.format_exc() +
                    "```")


def main():
    """Create and run the nmea_tcp_driver ROS node.

    Creates a ROS NMEA Driver and feeds it NMEA sentence strings from a TCP socket.

    ROS parameters:
        ~ip (str): IPV4 address of the socket to open.
        ~port (int): Local port of the socket to open.
        ~timeout (float): The time out period for the socket, in seconds.
    """
    rospy.init_node('nmea_tcp_driver')

    try:
        local_ip = rospy.get_param('~ip', '192.168.1.110')
        local_port = rospy.get_param('~port', 9904)
        timeout = rospy.get_param('~timeout_sec', 2)
    except KeyError as e:
        rospy.logerr("Parameter %s not found" % e)
        sys.exit(1)

    # Create a socket
    server = socketserver.TCPServer((local_ip, local_port), NMEAMessageHandler,
                                    bind_and_activate=False)
    server.frame_id = RosNMEADriver.get_frame_id()
    server.driver = RosNMEADriver()

    # Start listening for connections
    server.server_bind()
    server.server_activate()

    # Handle incoming connections until ROS shuts down
    try:
        while not rospy.is_shutdown():
            rlist, _, _ = select.select([server], [], [], timeout)
            if server in rlist:
                server.handle_request()
    except Exception:
        rospy.logerr(traceback.format_exc())
    finally:
        server.server_close()

if __name__ == "__main__":
    main()