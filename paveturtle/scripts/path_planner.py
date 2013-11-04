#!/usr/bin/env python
import rospy
from math import sqrt, atan
from turtlesim.msg import Pose
from paveturtle.msg import Point
from paveturtle.msg import Plan

class PathPlanner():
    def __init__(self):
        rospy.loginfo('launched path_planner')
        rospy.init_node('path_planner')
        self.xgo, self.ygo = 0, 0
        rospy.Subscriber('goto', Point, planner.missionsays)
        self.pub = rospy.Publisher('plan', Plan)
        rospy.Subscriber('pose', Pose, planner.turtlesays)
    def missionsays(self, dest):
        self.xgo, self.ygo = dest.x, dest.y
        rospy.loginfo('headed toward (%f,%f)' % (self.xgo, self.ygo))
    def turtlesays(self, pose):
        rospy.loginfo('turtle now at (%f,%f) angle %f' % (pose.x, pose.y, pose.theta))
        dx, dy = self.xgo - pose.x, self.ygo - pose.y
        dist = sqrt(dx * dx + dy * dy)
        angle = atan (dy / dx) - pose.theta
        if angle > 2: angle = 2.0
        elif angle < -2: angle = -2.0
        plan = Plan()
        plan.distance, plan.angle = dist, angle
        self.pub.publish(plan)
        rospy.loginfo('distance left %f, target angle %f' % (dist, angle))

if __name__ == '__main__':
    planner = PathPlanner()
    rospy.spin()