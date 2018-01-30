In this lab, you will create high-level methods for controlling the robot's linear and angular motion.
Instead of specifying an instantaneous velocity, you will use the robot's odometry information to move the robot a particular distance.

The Fetch robot uses a differential drive base, meaning that it can rotate in place and drive forward and backward.
However, it cannot move sideways very easily.
We will decompose the robot's movement into linear and angular movements.

In both cases, you will need to implement roughly the same strategy:
- Make the `Base` class subscribe to the `odom` topic.
- While the robot has not been moved by the desired amount, send velocity commands to move it in the correct direction.

# Make `Base` subscribe to odometry
First, modify `base.py` so that `Base` subscribes to odometry information.
You can put the subscriber in the `__init__` method and assign a method as a callback like so:
```py
class Base(object):
    def __init__(self):
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)

    def _odom_callback(self, msg):
        # TODO: do something
```

# Linear movement
To move linearly, add a method to `Base` called `go_forward`:
```py
def go_forward(self, distance, speed=0.1):
    """Moves the robot a certain distance.

    It's recommended that the robot move slowly. If the robot moves too
    quickly, it may overshoot the target. Note also that this method does
    not know if the robot's path is perturbed (e.g., by teleop). It stops
    once the distance traveled is equal to the given distance or more.

    Args:
        distance: The distance, in meters, to move. A positive value
            means forward, negative means backward.
        speed: The speed to travel, in meters/second.
    """
    # TODO: rospy.sleep until the base has received at least one message on /odom
    # TODO: record start position, use Python's copy.deepcopy
    start = copy.deepcopy(LATEST_ODOM)
    rate = rospy.Rate(10)
    # TODO: CONDITION should check if the robot has traveled the desired distance
    # TODO: Be sure to handle the case where the distance is negative!
    while CONDITION:
        # TODO: you will probably need to do some math in this loop to check the CONDITION
        direction = -1 if distance < 0 else 1
        self.move(direction * speed, 0)
        rate.sleep()
```

# Angular movement
Turning is very similar to going forward.
However, you also need to consider what happens if the given angle is greater than 2*pi or less than -2*pi.
You can use Python's modulo operator to deal with this.

Create a method called `turn`:
```py
def turn(self, angular_distance, speed=0.5):
    """Rotates the robot a certain angle.

    Args:
        angular_distance: The angle, in radians, to rotate. A positive
            value rotates counter-clockwise.
        speed: The angular speed to rotate, in radians/second.
    """
    # TODO: rospy.sleep until the base has received at least one message on /odom
    # TODO: record start position, use Python's copy.deepcopy
    start = copy.deepcopy(LATEST_ODOM)
    # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
    rate = rospy.Rate(10)
    # TODO: CONDITION should check if the robot has rotated the desired amount
    # TODO: Be sure to handle the case where the desired amount is negative!
    while CONDITION:
        # TODO: you will probably need to do some math in this loop to check the CONDITION
        direction = -1 if angular_distance < 0 else 1
        self.move(0, direction * speed)
        rate.sleep()
```

**Hints:**
This part is a bit confusing because it's not as easy to compute the rotation remaining, due to the "wraparound" issue.

Here are some helpful facts:
- `math.atan2` returns values in the range [-pi, pi]
- `x % (2*math.pi)` will always be in the range [0, 2*math.pi]

Let's assume that you can get your current yaw angle and your desired yaw angle in the range [0, 2*math.pi].
How can we compute the remaining angle (assume the remaining angle should be a positive value and we store the direction separately)?
Then, you will need to handle the following 4 cases:
1. Current to goal is counter-clockwise (CCW), no no wraparound in between
1. Current to goal is CCW, with wraparound in between
1. Current to goal is CW, no wraparound in between
1. Current to goal is CW, with wraparound in between

![untitled drawing 3](https://cloud.githubusercontent.com/assets/1175286/25262716/d5c6c4f4-260e-11e7-85b4-6ef781f92892.png)

In the first case, computing the remaining distance to go CCW is easy: goal - current.
However, this doesn't appear to work for the second case: goal - current = -270, but the remaining CCW distance should be 90.
How can you modify this formula to work for both the first and second case?
Once you have this formula, notice that the third and fourth cases are the same as the first two, but with the goal and current positions flipped.

# Test your odometry controllers
You can use this demo, which you should place in `applications/scripts/base_demo.py`:

```py
#! /usr/bin/env python

import math
import fetch_api
import rospy


def wait_for_time():                                                                          
    """Wait for simulated time to begin.
    """                                                                                       
    while rospy.Time().now().to_sec() == 0:                                                   
        pass


def print_usage():                                                                            
    print 'Usage: rosrun applications base_demo.py move 0.1'                                  
    print '       rosrun applications base_demo.py rotate 30'                                 
        
        
def main():
    rospy.init_node('base_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 3:
        print_usage() 
        return
    command = argv[1]
    value = float(argv[2])                                                                    
    
    base = fetch_api.Base()
    if command == 'move':                                                                     
        base.go_forward(value)
    elif command == 'rotate':                                                                 
        base.turn(value * math.pi / 180)                                                      
    else:
        print_usage()


if __name__ == '__main__':
    main()
```

## Accounting for drift
You may notice that the robot will drift off-course if it drives for long enough.
This drift may be caused by the floor being slippery or bumpy, inaccurate wheel encoder estimates, or overshooting.

One mitigation is to slow the robot down as it approaches the goal (either for turning or for driving forward).
A simple way to do this is to scale the speed linearly with the remaining distance, but with upper and lower bounds:
```
linear_speed = max(0.05, min(0.5, remaining_distance))
angular_speed = max(0.25, min(1, remaining_angle))
```

However, fundamentally, this control strategy is based on odometry from the wheel sensors, which will eventually be inaccurate.
A better approach is to have additional, more reliable sensors that tell us where the robot is.
We will see how the Fetch does navigation in the next lab.

# Interactive marker interface
As part of your assignment, you drive the robot by clicking on interactive markers.
Add interactive markers (the exact shape is up to you) around the robot's base.
Clicking on one of the markers should drive the robot forward by a half meter.
Clicking on two others should turn the robot by some amount (e.g., 30 degrees) clockwise or counter-clockwise.
The exact interface is up to you, but the markers would preferably follow the robot as it drives.