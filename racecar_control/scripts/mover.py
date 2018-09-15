#!/usr/bin/env python

import rospy

from ackermann_msgs.msg import AckermannDriveStamped


def mover():
    speed = 0.5
    turn = 0.25

    pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        x = 1
        th = 0

        msg.drive.speed = x * speed
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = th * turn
        msg.drive.steering_angle_velocity = 1

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
