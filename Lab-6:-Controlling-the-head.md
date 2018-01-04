In this lab, you will implement head movement on the Fetch.

# Read the Head Interface docs
The head interface section of the Fetch docs explains that there are two ways to control the head.
The first is to specify a point in space to look at.
The second is to control the pan and tilt angles of the head directly.

# Figure out the joint limits
Look at the "Robot Hardware Overview" section of the Fetch docs.
As we did in the previous labs, we will look up the joint limits and maximum velocity of the pan and tilt angles.

* What are the joint limits of the pan/tilt joints?
* What is the maximum angular speed of the pan/tilt joints?

Based on the joint limits and the maximum angular speed, 2.5 seconds is a reasonable, conservative duration for a head trajectory.

# Implement a demo
In this lab, we will go through the steps of implementing a demo for head movement.

Create a file called `head_demo.py` in `applications/scripts`.

Here is some starter code for the demo file:
```py
#! /usr/bin/env python

import rospy


def print_usage():
    print 'Usage:'
    print '    rosrun applications head_demo.py look_at FRAME_ID X Y Z'
    print '    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG'
    print 'Examples:'
    print '    rosrun applications head_demo.py look_at base_link 1 0 0.3'
    print '    rosrun applications head_demo.py pan_tilt 0 0.707'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('head_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        frame_id, x, y, z = argv[2], float(argv[3]), float(argv[4]), float(
            argv[5])
        rospy.logerr('Not implemented.')
    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        pan, tilt = float(argv[2]), float(argv[3])
        rospy.logerr('Not implemented.')
    else:
        print_usage()


if __name__ == '__main__':
    main()
```

As usual, you will need to complete the implementation.

## What's the difference between `applications` and `fetch_api`?
Notice that our repo splits code into library code (`fetch_api`) and application code (`applications`).
A file like `head_demo.py` is an application because it has a `main` function.
In general, we would like our applications to be small, and for them to mostly call out to library code.
A common convention is to place Python application code in a folder named `scripts`, while modules are placed in `src`.

## How do applications know about files in `fetch_api`?
The demo file imports code from the `fetch_api` module.
Normally, in Python, when you import a module, it is pre-installed on the system, or the file by the same name is adjacent to the current file.
So how do our demo files know where `fetch_api` is, when there is no `fetch_api.py` adjacent to our demo files?
This is something that catkin handles.
Our course repo already configures catkin properly, but in the future, we will show you how to configure catkin in this way.

## How do I extend `fetch_api`?
The `fetch_api` module is defined in `fetch_api/src/fetch_api`.
The actual content of the `fetch_api` module is explicitly listed in `fetch_api/src/fetch_api/__init__.py`.
For a class like `Head` to be part of the `fetch_api` module, we must import it in `__init__.py`.

# Implement head movement
Add the following starter code to `fetch_api/src/fetch_api/head.py`:
```py
#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import math
import rospy

LOOK_AT_ACTION_NAME = ''  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = ''  # TODO: Get the name of the pan/tilt action
PAN_JOINT = ''  # TODO: Get the name of the head pan joint
TILT_JOINT = ''  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = None  # TODO: Minimum pan angle, in radians.
    MAX_PAN = None  # TODO: Maximum pan angle, in radians.
    MIN_TILT = None  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = None  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        # TODO: Wait for both servers
        pass

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        # TODO: Send the goal
        # TODO: Wait for result
        rospy.logerr('Not implemented.')

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        # TODO: Create a trajectory point
        # TODO: Set positions of the two joints in the trajectory point
        # TODO: Set time of the trajectory point

        # TODO: Create goal
        # TODO: Add joint names to the list
        # TODO: Add trajectory point created above to trajectory

        # TODO: Send the goal
        # TODO: Wait for result

        rospy.logerr('Not implemented.')
```

**Implementation hints:**
* It is common to find "Stamped" messages in ROS. This means that the message includes a field of type `std_msgs/Header`. This is where you specify the `frame_id`: in the `header`.

Now, add the `Head` class to the `fetch_api` module by editing `fetch_api/src/fetch_api/__init__.py` and adding the following:
```
from .head import Head
```

# Run the demo
```
rosrun applications head_demo.py look_at base_link 1 0 0.3
rosrun applications head_demo.py pan_tilt 0 0
```

Positive `x`, `y`, and `z` are forward, left, and up, respectively.
The `base_link` is in the center of the robot's base, on the ground.
So x=1, y=0, z=0.3 in `base_link` means 1 meter straight ahead, 30 cm above the ground.