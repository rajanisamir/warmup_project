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

class PersonFollower(object):
    """ This node instructs the robot follow a person around the room. """

    # Constructor parameters
    #   follow_distance: how close we will get to the person.
    #   k_p_ang:         proportional control factor for angular velocity
    #   k_p_lin:         proportional control factor for linear velocity
    #   drive_angle:     error angle at which the robot will begin moving linearly
    def __init__(self, follow_distance=0.3, k_p_ang=0.03, k_p_lin=0.25, drive_angle=45):        
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

        # Initialize constructor parameters.
        self.k_p_ang = k_p_ang
        self.k_p_lin = k_p_lin
        self.follow_distance = follow_distance
        self.drive_angle = drive_angle

    # Determine nearest object by looking at scan data from all angles,
    #   set velocity based on that information, and publish to cmd_vel.
    def process_scan(self, data):
        # Initialize nearest_index and nearest_distance, which will keep
        #   track of the angle and distance to the object nearest to the
        #   robot.
        nearest_index = -1
        nearest_distance = 10 # Maximum LiDAR distance is 4.1m, so init value is safe.

        # Iterate through data.ranges, which contains the nearest object
        #   at each degree increment. If the value is non-zero, there is
        #   an object in that direction.
        for i in range(360):
            if data.ranges[i] > 0.0 and data.ranges[i] < nearest_distance:
                nearest_index = i
                nearest_distance = data.ranges[i]

        # If no object was found within the LiDAR range, stop the robot.
        if (nearest_distance == 10):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            return

        # Calculate the discrepancy between the robot's distance and angle
        #   and the desired follow distance and the angle (facing the person),
        #   respectively. The ternary operator is used to convert the range
        #   of angles 180-360 to -180-0, so that proportional control makes
        #   the robot turn in the correct direction.
        error_distance = nearest_distance - self.follow_distance
        error_angle = nearest_index if nearest_index < 180 else nearest_index - 360

        # Set linear and angular velocity based on the proportional control
        #   mechanism. Only set a non-zero linear velocity if the robot is
        #   facing roughly in the correct direction.
        self.twist.angular.z = self.k_p_ang * error_angle
        if (abs(error_angle) < self.drive_angle):
            self.twist.linear.x = self.k_p_lin * error_distance
        else:
            self.twist.linear.x = 0

        # Clamp angular velocity to 1.82 rad/s and linear velocity to 0.26 m/s
        #   to ensure we don't exceed the maximum velocity of the Turtlebot.
        self.twist.angular.z = min(1.82, self.twist.angular.z)
        self.twist.linear.x = min(0.26, self.twist.linear.x)
    
        # Publish Twist message to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node with default constructor parameters and run it.
    node = PersonFollower()
    node.run()