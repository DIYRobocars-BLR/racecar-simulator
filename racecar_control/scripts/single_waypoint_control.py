#!/usr/bin/env python

import cv2
import numpy
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import OccupancyGrid

class SingleWaypointControl(object):

    def rgb_callback(self, data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')

        # find the colors within the specified boundaries and create a mask out of it
        lower = numpy.array((180,0,0), dtype='uint8')
        upper = numpy.array((255,50,50), dtype='uint8')
        mask = cv2.inRange(cv_image, lower, upper)

        # apply the mask to the image
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        masked_image_msg = bridge.cv2_to_imgmsg(output, 'rgb8')
        self.segmented_img_pub.publish(masked_image_msg)

        #rospy.loginfo(rospy.get_caller_id() + ": Recieved RGB img")

    def point_cloud_callback(self, data):
        # point_list = []
        # for data in pc2.read_points(data, skip_nans=True):
        #     point_list.append([data[0], data[1], data[2], data[3]])
        pass

    def depth_callback(self, data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, '32FC1')
        distance_from_obstacle =  cv_image[0][0]
        self.dist_from_obstance.publish(distance_from_obstacle)

    def projected_map_callback(self, data):
        pass

    def __init__(self):
        speed = 0.5
        turn = 0.26

        self.pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped,
                                   queue_size=10)
        self.dist_from_obstance = rospy.Publisher("/single_waypoint_control/distance_from_obstacle", Image, queue_size=10)
        self.segmented_img_pub = rospy.Publisher("/single_waypoint_control/segmented_image", Image, queue_size=10)
        rospy.Subscriber("/camera/zed/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/projected_map", OccupancyGrid, self.projected_map_callback)
        rospy.Subscriber("/camera/zed/depth_registered/points", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("/camera/zed/depth/camera_info", CameraInfo, self.depth_callback)
        rospy.init_node('SingleWaypointControl', anonymous=False)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            msg = AckermannDriveStamped()
            x = 1
            th = 0

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
