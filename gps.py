#!/usr/bin/env python3
# The configuration assumes Python3.8 is being used. Enforce the correct version
import rospy
from std_msgs.msg import String,Bool

from ublox_gps import UbloxGps
import serial

def run_gps():
    pub = rospy.Publisher('gps_coordinates', String, queue_size=10)
    rospy.Subscriber('shutdown', Bool, shut_down)
    rospy.init_node('gps_pub_node', anonymous=True)
    rospy.loginfo("GPS node started")
    # initalise gps
    port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=10)
    gps = UbloxGps(port)

    # while the node is running
    while not rospy.is_shutdown():
        geo = gps.geo_coords()
        # publish them
        coord = str(geo.lat) + "#" + str(geo.lon)
        pub.publish(coord)
        print(coord + "\n")


def shut_down(data):
    if not data.data:
        rospy.signal_shutdown("shutdown called ")


if __name__ == '__main__':
    try:
        run_gps()
    except rospy.ROSException:
        pass
