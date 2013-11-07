#!/usr/bin/env python
import rospy
from math import sin, cos
from geometry_msgs.msg import Twist
from paveturtle.msg import Plan

class PathTracker():
    def __init__(self):
        rospy.init_node('path_tracker')
        rospy.loginfo('launched path_tracker')
        self.pub = rospy.Publisher('turtle1/cmd_vel', Twist)
        rospy.Subscriber('plan', Plan, self.plannersays)
    def plannersays(self, plan):
        cmdvel = Twist()
        if plan.distance > 2: cmdvel.linear.x = 2
        else: cmdvel.linear.x = plan.distance
        if plan.angle > 2: cmdvel.angular.z = 2
        elif plan.angle < -2: cmdvel.angular.z = -2
        else: cmdvel.angular.z = plan.angle
        self.pub.publish(cmdvel)
        rospy.sleep(1.0)
        rospy.loginfo('linear v (%f,%f,%f)' % (cmdvel.linear.x, cmdvel.linear.y, cmdvel.linear.z))
        rospy.loginfo('angular v (%f,%f,%f)' % (cmdvel.angular.x, cmdvel.angular.y, cmdvel.linear.z))

if __name__ == '__main__':
    tracker = PathTracker()
    rospy.spin()
