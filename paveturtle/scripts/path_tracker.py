#!/usr/bin/env python
import rospy
from math import sin, cos
from geometry_msgs.msg import Twist
from paveturtle.msg import Plan

class PathTracker():
    def __init__(self):
        rospy.init_node('path_tracker')
        rospy.loginfo('launched path_tracker')
        rospy.Subscriber('plan', Plan, tracker.plannersays)
        self.pub = rospy.Publisher('cmd_vel', Twist)
    def plannersays(self, plan):
        if plan.distance > 2: dist = 2
        else: dist = plan.distance
        if plan.angle > 2: ang = 2
        else: ang = plan.angle
        cmdvel = Twist()
        cmdvel.linear.x = dist * cos(ang)
        cmdvel.linear.y = dist * sin(ang)
        cmdvel.angular.x = ang
        self.pub.Publish(cmdvel)
        rospy.loginfo('turtle linear velocity (%f,%f,%f)' % cmdvel.linear)
        rospy.loginfo('turtle angular velocity (%f,%f,%f)' % cmdvel.angular)

if __name__ == '__main__':
    tracker = PathTracker()
    rospy.spin()
