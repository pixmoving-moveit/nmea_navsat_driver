# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
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

"""Provides a driver for NMEA GNSS devices."""
import math
import numpy as np
import rospy
from std_msgs.msg import Float32

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu
from geometry_msgs.msg import TwistStamped
# from autoware_sensing_msgs.msg import GnssInsOrientationStamped

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser
from tf.transformations import quaternion_from_euler
from ublox_msgs.msg import NavPVT

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


class RosNMEADriver(object):
    """ROS driver for NMEA GNSS devices."""

    def __init__(self):
        """Initialize the ROS NMEA driver.

        Creates the following ROS publishers:
            NavSatFix publisher on the 'fix' channel.
            TwistStamped publisher on the 'vel' channel.
            QuaternionStamped publisher on the 'heading' channel.
            TimeReference publisher on the 'time_reference' channel.

        Reads the following ROS parameters:
            ~time_ref_source (str): The name of the source in published TimeReference messages. (default None)
            ~useRMC (bool): If true, use RMC NMEA messages. If false, use GGA and VTG messages. (default False)
            ~epe_quality0 (float): Value to use for default EPE quality for fix type 0. (default 1000000)
            ~epe_quality1 (float): Value to use for default EPE quality for fix type 1. (default 4.0)
            ~epe_quality2 (float): Value to use for default EPE quality for fix type 2. (default (0.1)
            ~epe_quality4 (float): Value to use for default EPE quality for fix type 4. (default 0.02)
            ~epe_quality5 (float): Value to use for default EPE quality for fix type 5. (default 4.0)
            ~epe_quality9 (float): Value to use for default EPE quality for fix type 9. (default 3.0)
        """
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.imu_pub = rospy.Publisher('chc/imu', Imu, queue_size=1)
        self.imu_msg = Imu()

        self.pub_pitch = rospy.Publisher('chc/pitch',Float32,  queue_size=2)
        self.pitch_msg = Float32()
        self.data = []

        # self.pub_orientation = self.Publisher('/autoware_orientation', GnssInsOrientationStamped, 2)
        # self.orientation_msg = GnssInsOrientationStamped()

        self.ublox_navpvt_pub = rospy.Publisher('/fixposition/navpvt', NavPVT, queue_size=1, latch=False)
        self.use_GNSS_time = rospy.get_param('~use_GNSS_time', False)
        if not self.use_GNSS_time:
            self.time_ref_pub = rospy.Publisher(
                'time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)
        self.valid_fix = False

    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        """Public method to provide a new NMEA sentence to the driver.

        Args:
            nmea_string (str): NMEA sentence in string form.
            frame_id (str): TF frame ID of the GPS receiver.
            timestamp(rospy.Time, optional): Time the sentence was received.
                If timestamp is not specified, the current time is used.

        Returns:
            bool: True if the NMEA string is successfully processed, False if there is an error.
        """
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug(
                "Failed to parse NMEA sentence. Sentence was: %s" %
                nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        if not self.use_GNSS_time:
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            if self.time_ref_source:
                current_time_ref.source = self.time_ref_source
            else:
                current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            data = parsed_sentence['GGA']

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            gps_qual = data['fix_type']
            if gps_qual == 0:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif gps_qual == 1:
                current_fix.status.status = NavSatStatus.STATUS_FIX
            elif gps_qual == 2:
                current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif gps_qual in (4, 5):
                current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            elif gps_qual == 9:
                # Support specifically for NOVATEL OEM4 recievers which report WAAS fix as 9
                # http://www.novatel.com/support/known-solutions/which-novatel-position-types-correspond-to-the-gga-quality-indicator/
                current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            else:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX

            if gps_qual > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            current_fix.header.stamp = current_time

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            hdop = data['hdop']
            current_fix.position_covariance[0] = hdop ** 2
            current_fix.position_covariance[4] = hdop ** 2
            current_fix.position_covariance[8] = (2 * hdop) ** 2  # FIXME
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            self.fix_pub.publish(current_fix)

            if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                current_time_ref.time_ref = rospy.Time(
                    data['utc_time'][0], data['utc_time'][1])
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as
            # well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                    current_time_ref.time_ref = rospy.Time(
                        data['utc_time'][0], data['utc_time'][1])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide
            # it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'CHC' in parsed_sentence:
            data = parsed_sentence['CHC']
            if data['fix_valid']:
                navpvt_msg = NavPVT()
                navpvt_msg.heading = int(math.degrees(data['heading'])*100000)
                self.ublox_navpvt_pub.publish(navpvt_msg)
            try:
                self.pitch_msg.data = data["pitch"]
                self.data.append(data["pitch"])
                self.pub_pitch.publish(self.pitch_msg)

                self.imu_msg.header.stamp = rospy.Time()
                self.imu_msg.header.frame_id = "gnss"
                # orientation
                heading = math.radians(90.0-data['heading'])
                pitch = math.radians(data['pitch'])
                roll = math.radians(data['roll'])
                [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
                self.imu_msg.orientation.x = qx
                self.imu_msg.orientation.y = qy
                self.imu_msg.orientation.z = qz
                self.imu_msg.orientation.w = qw
                # angular velocity the coordinate of imu is y-front x-right z-up, 
                # so it has to be converted to right-handed coordinate
                self.imu_msg.angular_velocity.x = math.radians(data["angular_velocity_y"])
                self.imu_msg.angular_velocity.y =  math.radians(-data["angular_velocity_x"])
                self.imu_msg.angular_velocity.z =  math.radians(data["angular_velocity_z"])

                self.imu_msg.linear_acceleration.x = data["linear_acceleration_y"]
                self.imu_msg.linear_acceleration.y = -data["linear_acceleration_x"]
                self.imu_msg.linear_acceleration.z = data["linear_acceleration_z"]

                self.imu_msg.angular_velocity_covariance[0] = 0.001
                self.imu_msg.angular_velocity_covariance[4] = 0.001
                self.imu_msg.angular_velocity_covariance[8] = 0.001
                self.imu_pub.publish(self.imu_msg) 
                # orientation msg of autoware
                # self.orientation_msg.header = self.imu_msg.header
                # self.orientation_msg.orientation.orientation = self.imu_msg.orientation
                # self.orientation_msg.orientation.rmse_rotation_x = 0.001745329 # 0.1 degree
                # self.orientation_msg.orientation.rmse_rotation_y = 0.001745329 # 0.1 degree 
                # self.orientation_msg.orientation.rmse_rotation_z = 0.001745329 # 0.1 degree
                # self.pub_orientation.publish(self.orientation_msg)
            except UnicodeDecodeError as err:
                self.get_logger().warn("UnicodeDecodeError: {0}".format(err))
        else:
            return False

    @staticmethod
    def get_frame_id():
        """Get the TF frame_id.

        Queries rosparam for the ~frame_id param. If a tf_prefix param is set,
        the frame_id is prefixed with the prefix.

        Returns:
            str: The fully-qualified TF frame ID.
        """
        frame_id = rospy.get_param('~frame_id', 'gps')
        if frame_id[0] != "/":
            # Add the TF prefix
            prefix = ""
            prefix_param = rospy.search_param('tf_prefix')
            if prefix_param:
                prefix = rospy.get_param(prefix_param)
                if prefix[0] != "/":
                    prefix = "/%s" % prefix
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
