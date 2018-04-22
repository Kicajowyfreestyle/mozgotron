#!/usr/bin/env python

import rospy
from math import atan2, sin, cos, pi
from geometry_msgs.msg import Twist, Vector3, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

dt = 0.01

# Robot dimensions
b = 0.07
r = 0.045

class DummyController:
    def __init__(self):
        self.prev_pose = Pose2D(0.0, 0.0, 0.0)
        self.pub = None

    def initPublisher(self):
        self.pub = rospy.Publisher('epuck_robot_0/cmd_vel', Twist, queue_size=10)

    def calculate(self, pose):
        dx = pose.x - self.prev_pose.x
        dy = pose.y - self.prev_pose.y

        mov = abs(dx + dy)
        print "mov", mov

        tan = atan2(dy, dx) - self.prev_pose.theta
        #if dy > 0 and pose.x > 0:
        #    tan = atan2(dy, dx) + self.prev_pose.theta
        #if dy > 0 and pose.x <= 0:
        #    tan = -atan2(dy, dx) + self.prev_pose.theta
        #if dy < 0 and pose.y > 0:
        #    tan = atan2(dy, dx) - self.prev_pose.theta
        #if dy > 0 and pose.y <= 0:
        #    tan = atan2(dy, dx) - self.prev_pose.theta

        #print "tanp", -atan2(dy, dx)
        #if tan < -pi:
        #    tan += pi
        #if tan > pi:
        #    tan -= pi
        print "tan", tan
        print "prev", self.prev_pose.theta

        w = 1 * -tan / abs(tan+1e-8)
        u = 0
        linear = Vector3(u, 0, 0)
        rotation = Vector3(0, 0, w)
        self.pub.publish(linear, rotation)
        rospy.sleep(0.33*2*abs(tan)/pi)

        w = 0
        u = 0.5
        linear = Vector3(u, 0, 0)
        rotation = Vector3(0, 0, w)
        self.pub.publish(linear, rotation)
        rospy.sleep(3.9*mov/0.5)

        w = 0
        u = 0
        linear = Vector3(u, 0, 0)
        rotation = Vector3(0, 0, w)
        self.pub.publish(linear, rotation)
        rospy.sleep(0.5)

        self.prev_pose.x = pose.x
        self.prev_pose.y = pose.y
        self.prev_pose.theta = atan2(dy, dx)

dummy = DummyController()

def callback(pose):
    dummy.calculate(pose)

if __name__ == '__main__':
    rospy.init_node('linearized_controller', anonymous=True)
    dummy.initPublisher()

    # Send initial message - inital position
    #rospy.Subscriber('odom', Odometry, callback)
    rospy.Subscriber('set_destination', Pose2D, callback)
    #rospy.Subscriber('set_destination', Pose2D, callbackDestination)

    rospy.spin()




