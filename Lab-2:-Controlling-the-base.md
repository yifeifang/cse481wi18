The Fetch Research Edition manual can be found at http://docs.fetchrobotics.com/.
All of its content is worth reading, but we are particularly interested in the [API overview](http://docs.fetchrobotics.com/api_overview.html).

# Read the **Base Interface** section
Read the **Base Interface** section, then answer these questions:

1. The documentation says that you can control the base by publishing to a topic. What is the type of the topic?
1. How do you look up the fields of the message type?
1. You can publish messages to one of two topics. Which one is better to publish to?

# Keyboard teleop
The starter code partially implements keyboard teleop for the Fetch, in `applications/scripts/keyboard_teleop.py`.
If you try running it, you will get lots of `Not implemented` errors:
```rosrun applications keyboard_teleop.py```

You will implement most of the robot's functionality in the `fetch_api` package.
Implement the base movement by editing `fetch_api/src/fetch_api/base.py`.
Note that the teleop app already reads keyboard input in an infinite loop and calls the `move` method inside the loop.
As a result, you only need to write code to publish the appropriate velocity command.
The solution should only be few lines of code.

Refer to the ROS tutorials to see how to write a ROS publisher in Python.