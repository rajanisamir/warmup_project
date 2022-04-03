#!/usr/bin/env python3

import rospy
import math

# messages needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node commands the Turtlebot to drive in a square path. """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # set up publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # straight_line_twist is a message that drives the robot in a straight line at its maximum velocity
        straight_line_twist = Twist(
            linear = Vector3(0.26, 0, 0),
            angular = Vector3()
        )
        # turn_twist is a message that rotates the robot at 90 degrees per second
        turn_twist = Twist(
            linear = Vector3(),
            angular = Vector3(0, 0, math.pi / 2)
        )

        # allow the publisher enough time to set up before publishing the first message
        rospy.sleep(1)

        # continuously drive a straight path for 4 seconds, then turn 90 degrees
        while not rospy.is_shutdown():
            self.robot_movement_pub.publish(straight_line_twist)
            rospy.sleep(4)
            # turn at 90 degrees per second for 1.04 seconds to offset friction
            self.robot_movement_pub.publish(turn_twist)
            rospy.sleep(1.04)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()