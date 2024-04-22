#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointcloud_callback(msg):
    rospy.loginfo("Received PointCloud2 message")

    # Extract information from the PointCloud2 message
    height = msg.height
    width = msg.width
    fields = msg.fields
    is_bigendian = msg.is_bigendian
    point_step = msg.point_step
    row_step = msg.row_step
    data = msg.data

    rospy.loginfo("Height: %d, Width: %d", height, width)
    rospy.loginfo("Fields: %s", fields)
    rospy.loginfo("Is Big Endian: %s", is_bigendian)
    rospy.loginfo("Point Step: %d", point_step)
    rospy.loginfo("Row Step: %d", row_step)

    # Iterate through the points in the data array
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    # for point in gen:
    #     rospy.loginfo("Point: x=%f, y=%f, z=%f", point[0], point[1], point[2])

def main():
    rospy.init_node('pointcloud_subscriber', anonymous=True)

    # Subscribe to the PointCloud2 topic
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloud_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
