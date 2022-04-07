#!/usr/bin/env python3
# TOPICS:
#   cmd_vel: publishing, used for setting robot velocity
#   scan   : subscribing, used to detect where the person is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to the person.
follow_distance = 0.3

class PersonFollower(object):
    """ This node follows a person around the room. """
    def __init__(self):
        # Start rospy node.
        rospy.init_node("follow_person")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        # Set proportional control factors for angular and linear velocity.
        self.k_p_ang = 0.03
        self.k_p_lin = 0.25

    def process_scan(self, data):
        # Determine nearest object by looking at scan data from all angles
        #   the robot, set velocity based on that information, and
        #   publish to cmd_vel.

        # Initialize nearest_index and nearest_distance, which will keep
        #   track of the angle and distance to the object nearest to the
        #   robot.
        nearest_index = -1
        nearest_distance = 10

        # Iterate through data.ranges, which contains the nearest object
        #   at each degree increment. If the value is non-zero, there is
        #   an object in that direction.
        #   TODO: ENSURE IT'S OKAY TO COMPARE FLOATS IN THIS WAY
        #   TODO: MAKE SURE IT STOPS WHEN THERE'S NOTHING AROUND
        for i in range(360):
            if data.ranges[i] > 0.0 and data.ranges[i] < nearest_distance:
                nearest_index = i
                nearest_distance = data.ranges[i]

        # Calculate the discrepancy between the robot's distance and angle
        #   and the desired follow distance and the angle (facing the person),
        #   respectively. The ternary operator is used to convert the range
        #   of angles 180-360 to -180-0, so that proportional control makes
        #   the robot turn in the correct direction.
        error_distance = nearest_distance - follow_distance
        error_angle = nearest_index if nearest_index < 180 else nearest_index - 360

        # Set linear and angular velocity based on the proportional control
        #   mechanism. Only set a non-zero linear velocity if the robot is
        #   facing roughly the right direction.
        self.twist.angular.z = self.k_p_ang * error_angle
        if (abs(error_angle) < 45):
            self.twist.linear.x = self.k_p_lin * error_distance
        else:
            self.twist.linear.x = 0
    
        # Publish Twist message to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()