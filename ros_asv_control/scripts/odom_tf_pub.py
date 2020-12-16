#!/usr/bin/env python
import time
import rospy
import tf
import tf_conversions
from math import cos, sin, pi

from geometry_msgs.msg import Vector3Stamped, Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry

WHEEL_BASE = 0.48


class Odometry_Publisher():
    def __init__(self):
        rospy.loginfo("Starting Odom Publisher")
        rospy.init_node("Odometry_Publisher")
        self.left_vel = 0
        self.right_vel = 0
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcastor = tf.TransformBroadcaster()
        rospy.Subscriber("speed", Vector3Stamped, callback=self.get_wheel_vel)

    def get_wheel_vel(self, vel):
        self.left_vel = vel.vector.x
        self.right_vel = vel.vector.y

    def publish_odom(self):
        x = 0.0
        y = 0.0
        th = 0.0

        rate = rospy.Rate(20)
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            dt = (current_time - last_time).to_sec()

            vx = (self.left_vel + self.right_vel)/2
            vy = 0
            vth = (self.right_vel - self.left_vel)/WHEEL_BASE

            dx = (vx * cos(th))*dt
            dy = (vy * sin(th))*dt
            dth = vth*dt

            x += dx
            y += dy
            th += dth

            odom_quat = tf_conversions.transformations.quaternion_from_euler(
                0, 0, th)

            self.odom_broadcastor.sendTransform(
                (x, y, 0.), odom_quat, current_time, "base_link", "odom")

            rospy.loginfo("Publishing odom")
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            self.odom_pub.publish(odom)
            rospy.loginfo("Publishing Pose")
            last_time = current_time
            rate.sleep()


if __name__ == "__main__":
    try:
        node = Odometry_Publisher()
        node.publish_odom()
    except rospy.ROSInterruptException:
        rospy.loginfo("Killing Odom Publisher")
