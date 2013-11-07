#!/usr/bin/env python
import rospy
import fileinput
import sys
from math import sqrt
from turtlesim.msg import Pose
from paveturtle.msg import Point, Plan

class MissionControl():
    def __init__(self, waypoints):
        rospy.init_node('mission_control')
        rospy.loginfo('launched mission_control')
        self.waypoint = waypoints
        self.pub = rospy.Publisher('goto', Point)
        self.dist = 0
        rospy.Subscriber('turtle1/pose', Pose, self.turtlesays)
        rospy.Subscriber('distance', Plan, self.plannersays)
    def turtlesays(self, pose):
        pt = self.waypoint[0]
        dx = pt[0] - pose.x
        dy = pt[1] - pose.y
        self.dist = sqrt(dx * dx + dy * dy)
    def plannersays(self, dist):
        pt = self.waypoint[0]
        if self.distance < 0.1:
            self.waypoint = self.waypoint[1:]
            if len(self.waypoint) == 0:
                rospy.signal_shutdown('reached all waypoints')
            else:
                rospy.loginfo('reached waypoint (%f,%f)' % pt)
                pt = self.waypoint[0]
        else:
            rospy.loginfo('headed toward (%f,%f)' % pt)
        point = Point()
        point.x, point.y = pt
        self.pub.publish(point)
        rospy.sleep(1.0)

if __name__== '__main__':
    waypoints = []
    waypoints.extend([(float(x[0]), float(x[1])) for x in (line.split() for line in fileinput.input())])
    fileinput.close()
    if len(waypoints) == 0:
        rospy.signal_shutdown('no coordinates provided by input file')
    else:
        mission = MissionControl(waypoints)
        rospy.spin()
