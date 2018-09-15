#!/usr/bin/env python

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image


def rgb_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "RGB: I heard %s", data.data)


def point_cloud_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "PointCloud: I heard %s", data.data)


def depth_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Depth: I heard %s", data.data)


def mover():
    speed = 0.5
    turn = 0.25

    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)
    rospy.Subscriber("/camera/zed/color/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera/zed/depth_registered/points", Image, point_cloud_callback)
    rospy.Subscriber("/camera/zed/depth/camera_info", Image, depth_callback)
    rospy.init_node('mover', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        msg = AckermannDriveStamped()

        x = 1
        th = 0.5

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.drive.speed = x * speed
        msg.drive.steering_angle = th * turn

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
