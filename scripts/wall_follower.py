#!/usr/bin/env python3
# TOPICS:
#   cmd_vel: publishing, used for setting robot velocity
#   scan   : subscribing, used to detect where the wall is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class WallFollower(object):
    """ This node follows a person around the room. """

    # Constructor parameters
    #   wall_distance:   the distance we should maintain to the wall
    #   k_p_dist:        proportional control factor controlling how much the distance
    #                      from the wall affects the desired angle
    #   k_p_ang:         proportional control factor controlling how much the error in
    #                      angle affects the angular velocity 
    def __init__(self, wall_distance=0.4, k_p_ang=0.01, k_p_dist=50):        
        # Start rospy node.
        rospy.init_node("follow_wall")

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
        self.wall_distance = wall_distance
        self.k_p_ang = k_p_ang
        self.k_p_dist = k_p_dist

    # Determine nearest object by looking at scan data from all angles
    #   the robot, set velocity based on that information, and
    #   publish to cmd_vel.
    def process_scan(self, data):
        # Initialize nearest_index and nearest_distance, which will keep
        #   track of the angle and distance to the object nearest to the
        #   robot.
        nearest_index = -1
        nearest_distance = 10

        # Iterate through data.ranges, which contains the nearest object
        #   at each degree increment. If the value is non-zero, there is
        #   an object in that direction.
        #   TODO: MAKE SURE IT STOPS WHEN THERE'S NOTHING AROUND
        for i in range(360):
            if data.ranges[i] > 0.0 and data.ranges[i] < nearest_distance:
                nearest_index = i
                nearest_distance = data.ranges[i]

        # Calculate the discrepancy between the robot's distance and angle
        #   and the desired wall distance and the angle (90 degrees),
        #   respectively. The ternary operator is used to convert the range
        #   of angles 90-270 to 0-180, and the range TODO so that proportional control makes
        #   the robot turn in the correct direction.

        # TODO: say clamp value
        error_distance = nearest_distance - self.wall_distance
        desired_angle = max(0, min(180, 90 - (nearest_distance - self.wall_distance) * self.k_p_dist))
        error_angle = (nearest_index - desired_angle if nearest_index < 270 else nearest_index - 450)

        print("Error distance: ", error_distance)
        print("Desired_angle: ", desired_angle)
        print("Error angle: ", error_angle)

        # Set velocity based on the proportional control mechanism.
        self.twist.angular.z = self.k_p_ang * error_angle
        self.twist.linear.x = 0.05
    
        # Publish Twist message to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node with default constructor parameters and run it.
    node = WallFollower()
    node.run()