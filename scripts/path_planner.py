#!/usr/bin/env python
import rospy
from math import atan
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D

class PathPlanner():
    destx, desty = 0, 0
    x, y, theta = 0, 0, 0
    def __init__(self):
        rospy.init_node('path_planner')
        rospy.loginfo('launched path_planner')
    def missionsays(self, dest):
        self.destx, self.desty = dest.x, dest.y
        rospy.loginfo('headed toward (%f,%f)' % (self.destx, self.desty))
    def turtlesays(self, pose):
        self.x, self.y, self.theta = pose.x, pose.y, pose.theta
        rospy.loginfo('turtle angle %f' % self.theta)
        rospy.loginfo('turtle velocity %f %f' % (pose.linear, pose.angular))
    def publish(self, pub):
        dx = self.destx - self.x
        dy = self.desty - self.y
        angle = atan(dy / dx) - self.theta
        if angle > 2: angle = 2
        elif angle < -2: angle = - 2
        pub.publish(Pose2D(dx, dy, angle))
        rospy.sleep(1.0)

if __name__ == '__main__':
    planner = PathPlanner()
    rospy.Subscriber('goto', Pose2D, planner.missionsays)
    rospy.Subscriber('pose', Pose, planner.turtlesays)
    rospy.spin()
    pub = rospy.Publisher('plan', Pose2D)
    try:
        while not rospy.is_shutdown():
            planner.publish(pub)
    except rospy.ROSInterruptException:
        pass
