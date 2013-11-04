#!/usr/bin/env python
import rospy
from math import sqrt, sin, cos
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

class PathTracker():
    dist = 0
    angle = 0
    def __init__(self):
        rospy.init_node('path_tracker')
        rospy.loginfo('launched path_tracker')
    def plannersays(self, data):
        self.dist = sqrt(data.x * data.x + data.y * data.y)
        self.angle = data.theta
        rospy.loginfo('distance left %f' % self.dist)
    def publish(self, pub):
        if self.dist > 2:
            vx = 2 * cos(self.angle)
            vy = 2 * sin(self.angle)
        else:
            vx = self.dist * cos(self.angle)
            vy = self.dist * cos(self.angle)
        pub.Publish((vx, vy, 0), (self.angle, 0, 0))
        rospy.sleep(1.0)

if __name__ == '__main__':
    tracker = PathTracker()
    rospy.Subscriber('plan', Pose2D, tracker.plannersays)
    rospy.spin()
    pub = rospy.Publisher('cmd_vel', Twist)
    try:
        while not rospy.is_shutdown():
            tracker.publish(pub)
    except rospy.ROSInterruptException:
        pass
