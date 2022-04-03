#!/usr/bin/env python3

import rospy
import math

# messages needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node commands the Turtlebot to drive in a square. """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # set up publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # set up the Twist message we want to send
        straight_line_twist = Twist(
            linear = Vector3(0.3, 0, 0),
            angular = Vector3()
        )
        turn_twist = Twist(
            linear = Vector3(),
            angular = Vector3(0, 0, math.pi / 2)
        )
        stop_twist = Twist(
            linear=Vector3(),
            angular=Vector3()
        )

        # allow the publisher enough time to set up before publishing the first message
        rospy.sleep(1)

        for i in range(4):
            # publish the message
            self.robot_movement_pub.publish(straight_line_twist)
            rospy.sleep(3)
            self.robot_movement_pub.publish(stop_twist)
            rospy.sleep(0.15)
            self.robot_movement_pub.publish(turn_twist)
            rospy.sleep(1)
            self.robot_movement_pub.publish(stop_twist)
            rospy.sleep(0.15)

        self.robot_movement_pub.publish(stop_twist)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()