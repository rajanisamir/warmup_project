# warmup_project

## Robot Behaviors
1) Driving in a Square
- The goal of this part of the project is to have the robot drive continuously in a square path. My approach was to break down the movement of the robot into steps consisting of driving in a straight line path, followed by turning by 90 degrees. Repeating this set of two actions in a loop will create a square path.
- The `DriveSquare` class consists of a constructor `__init__`, which initializes a node called `drive_square` and sets up a publisher for the `cmd_vel` topic. The `run` function initializes two messages, `straight_line_twist`, which commands the robot to drive in a straight line at its maximum velocity, and `turn_twist`, which commands the robot to rotate at π/2 radians per second. Then, a sleep call allows the publisher enough time to set up, and a while loop tells the robot to continuously drive in a straight line for four seconds, and then turn at 90 degrees per second for 1.04 seconds. Allowing the turn to occur for slightly longer than 1 second offsets the friction of the carpet on which this robot was tested.
- Gif: ![drive_square](gifs/drive_square.gif)

2) Person Follower
- The goal of this part of the project is to instruct the robot to follow a person around a room (while maintaining a safe distance). My approach was to use proportional control on the robot for setting both its linear velocity and its angular velocity. The robot uses LiDAR to detect the direction and distance to the nearest object. It uses a separate proportionality constant for both linear and angular velocity to set its speed; a greater distance to the person will cause the robot to move faster, and a greater angle to the person will cause the robot to turn faster. The robot will only begin to drive forward once the angle to the person is below a specified value, which has a default value of 45 degrees.
- The `PersonFollower` class consists of a constructor, `__init__`, which initializes a node called `follow_person` and sets up a subscriber for the `scan` topic (with the `process_scan` function registered as a callback) and a publisher to the `cmd_vel` topic. Both a default twist message and the parameters accepted by the constructor (such as the proportional control factors) are also initialized. The `process_scan` function is called when the subscriber receives a message on the `scan` topic. It first scans through `data.ranges` to find the distance and angle to the nearest object. If no object is found within the LiDAR range, the robot is stopped; otherwise, "error" terms are computed, which contain the differences between the distance to the nearest object and desired follow distance, and between the angle to the nearest object and the desired angle (in this case, the desired angle is zero, because we always want to face the person). A conversion is performed which changes the range of angles from 180-360 to -180-0, causing the proportional control mechanism to make the robot turn counterclockwise for a person to the left and clockwise for a person to the right. The linear velocity is set to 0 if the robot's angle to the nearest object is greater than `drive_angle`, a parameter whose default value is 45 degrees. The velocities are obtained by multiplying the error term by the corresponding proportionality factor, and they are clamped to the Turtlebot's maximum velocity before the `Twist` message is published.
- Gif: ![person_follower](gifs/person_follower.gif)

3) Wall Follower
- The goal of this part of the project is to instruct the robot to follow a wall while maintaining a specified distance. My approach was to use proportional control to set both the robot's desired angle and its angular velocity. The desired angle is tilted towards the wall if is too far, while it is tilted away from the wall if it is too close. Its angular velocity is set proportional to the difference between the robot's current angle to the wall and its desired angle.
- The `WallFollower` class consists of a constructor, `__init__`, which initializes a node called `follow_wall` and sets up a subscriber for the `scan` topic (with the `process_scan` function registered as a callback) and a publisher to the `cmd_vel` topic. Both a default twist message and the parameters accepted by the constructor (such as the proportional control factors) are also initialized. The `process_scan` function is called when the subscriber receives a message on the `scan` topic. It first scans through `data.ranges` to find the distance and angle to the nearest object. If no object is found within the LiDAR range, the robot is stopped. Otherwise, the difference between the robot's actual distance to the wall and its desired distance is computed, and the robot uses the proportionality constant `k_p_dist`, setting the angle to less than 90 degrees (by an amount proportional to the difference in distance) if it too far from the wall and greater than 90 degrees if it is too close to the wall. The robot then uses the proportionality `k_p_ang` to set the angular velocity proportional to the difference between the actual angle to the nearest object and the computed desired angle. The linear velocity is set to a constant, and the angular velocity is clamped to the Turtlebot's maximum velocity before the `Twist` message is published.
- Gif: ![wall_follower](gifs/wall_follower.gif)

## Challenges
One challenge I faced when programming the "Drive in a Square" behavior is that the friction of the carpet meant that the robot turned at slightly less than 90 degrees because the friction of the carpet meant its angular velocity was reduced. To compensate for this, I increased the turn time by 4%, which was sufficient to correct for the friction. When programming the "Person Follower" behavior, I ran into the issue that the robot would turn toward the person until it was within the specified drive angle (45 degrees), then begin driving, and then the angle to the person would increase such that it was no longer within the drive angle. This caused the robot to "jitter" when turning and driving towards a target. To solve this, I increased the proportionality factor for the angular velocity so that the robot turned fast enough to the point where this was no longer an issue. The "Wall Follower" required significant tweaking of the proportionality factors to prevent the robot from undercompensating or overcompensating when turning.

## Future Work
In the future, I could improve the "Drive in a Square" behavior by using the Turtlebot3's odometry instead of by using timing, which would allow turns to be executed with greater precision and remove the need for manual configuration of turn times on different surfaces. I could improve the "Person Follower" behavior by having the robot keep track of the last known position of a person, so that if the person walks outside of the LiDAR's maximum range, the robot would continue driving in that direction, rather than stopping. Finally, I could improve the "Wall Follower" behavior by allowing the robot to follow the wall on either side of its body, rather than just on one side.

## Takeaways
- The movement of a physical robot is not a perfect reflection of what would happen in a simulation. Factors like friction, noisy sensors, and surrounding objects must be taken into consideration, and it is important to test a robot in the same environment in which it will ultimately be deployed. Friction might mean that the robot's velocity is smaller than what the code expects, noisy sensors can mean that the a physical object that is present is not detected (and vice versa), and surrounding objects might throw off a program's assumptions (such as that a person is always the object nearest to it). Multiple trials of a robot's behavior in the same environment should also be performed when testing, because factors like network speed (which could affect the amount of time it takes to send a message) can vary across trials.
- The constants associated with exercising proportional control require fine tuning. Large proportionality constants tend to lead to overcompensation, in which the robot adjusts its behavior by a large amount based on a measurement, and by the next measurement, it must reverse its behavior. For example, if the robot tries to rotate to face an object, it could rotate too much to the left, then turn too much to the right to compensate, then turn back to the left, and so on. However, small proportionality constants can mean the robot will take far too long to reach the desired state. It is important to balance these two factors to arrive at an appropriate value.
