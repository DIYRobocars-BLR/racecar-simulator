#!/usr/bin/env python

import rospy

from ackermann_msgs.msg import AckermannDriveStamped


def mover():
    speed = 0.5
    turn = 0.25

    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)
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
