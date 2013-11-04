#!/usr/bin/env python
import rospy
import fileinput
from math import sqrt
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D

class MissionControl():
    waypoint = []
    currpt = Pose2D(0,0,0)
    index = 0
    def __init__(self, waypoints):
        rospy.init_node('mission_control')
        rospy.loginfo('launched mission_control')
        self.waypoint = waypoints
    def listen(self, pose):
        rospy.loginfo('turtle position (%f,%f)' % (pose.x, pose.y))
        self.currpt = Pose2D(pose.x, pose.y, 0)
    def distance(self):
        waypt = self.waypoint[self.index]
        dx = self.currpt.x - waypt.x
        dy = self.currpt.x - waypt.y
        return sqrt(dx * dx + dy * dy)
    def publish(self, pub):
        if self.distance() < 0.1:
            pt = self.waypoint[self.index]
            rospy.loginfo('waypoint (%f,%f) was reached' % (pt.x, pt.y))
            self.index += 1
        pub.publish(self.waypoint[self.index])
        rospy.sleep(1.0)

if __name__== '__main__':
    waypoints = []
    for line in fileinput.input():
        coordinate = line.split()
        waypoints.append(Pose2D(coordinate[0], coordinate[1], 0))
    mission = MissionControl(waypoints)
    rospy.Subscriber('pose', Pose, mission.listen)
    rospy.spin()
    pub = rospy.Publisher('goto', Pose2D)
    try:
        while not rospy.is_shutdown():
            mission.publish(pub)
    except rospy.ROSInterruptException:
        pass
