# warmup_project

## Robot Behaviors
1) Driving in a Square
- The goal of this part of the project is to have the robot drive continuously in a square path. My approach was to break down the movement of the robot into steps consisting of driving in a straight line path, followed by turning by 90 degrees. Repeating this set of two actions in a loop will create a square path.
- The `DriveSquare class` consists of a constructor `__init__`, which initializes a node called `drive_square` and sets up a publisher for the `cmd_vel` topic. The `run` function initializes two messages, `straight_line_twist`, which commands the robot to drive in a straight line at its maximum velocity, and `turn_twist`, which commands the robot to rotate at pi/2 radians per second. Then, a sleep call allows the publisher enough time to set up, and a while loop tells the robot to continuously drive in a straight line for four seconds, and then turn at 90 degrees per second for 1.04 seconds. Allowing the turn to occur for slightly longer than 1 second offsets the friction of the carpet on which this robot was tested.
- Gif: ![drive_square_new](https://user-images.githubusercontent.com/38731359/161453305-6c3828db-bc07-49b0-a558-5ed07441291e.gif)
