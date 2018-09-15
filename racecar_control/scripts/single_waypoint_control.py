#!/usr/bin/env python

import cv2
import numpy
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo


class SingleWaypointControl(object):

    def rgb_callback(self, data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
        boundaries = [
            ([200, 2, 2], [255, 2, 2])
        ]
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = numpy.array(lower, dtype='uint8')
            upper = numpy.array(upper, dtype='uint8')

            # find the colors within the specified boundaries and create a mask out of it
            mask = cv2.inRange(cv_image, lower, upper)
            # apply the mask to the image
            output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            masked_image_msg = bridge.cv2_to_imgmsg(output, 'rgb8')
            self.segmented_img_pub.publish(masked_image_msg)

        rospy.loginfo(rospy.get_caller_id() + "RGB: I heard %s", cv_image)

    def point_cloud_callback(self, data):
        pass

    def depth_callback(self, data):
        pass

    def __init__(self):
        speed = 0.5
        turn = 0.25

        self.pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped,
                                   queue_size=10)
        self.segmented_img_pub = rospy.Publisher("/single_waypoint_control/segmented_image", Image, queue_size=10)
        rospy.Subscriber("/camera/zed/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/zed/depth_registered/points", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("/camera/zed/depth/camera_info", CameraInfo, self.depth_callback)
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
            self.pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        SingleWaypointControl()
    except rospy.ROSInterruptException:
        pass
