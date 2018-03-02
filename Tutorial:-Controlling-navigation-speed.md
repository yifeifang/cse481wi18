The navigation speed of the robot is specified through a couple of launch files.
In order to modify these launch files, you will need to create a copy of them and run those copies instead.

# Make copies of the launch files
In addition to copying the main `fetch_nav.launch` file, we need to copy an additional file that is included by `fetch_nav.launch`.
You can see what files are included in `fetch_nav.launch` by running `rosed fetch_navigation fetch_nav.launch`.

```bash
cd fetch_api/launch
roscp fetch_navigation fetch_nav.launch .
roscp fetch_navigation move_base.launch.xml .
ls
```

You should see two new files: fetch_nav.launch and move_base.launch.xml.

# Make a copy of the config files
If you look at `move_base.launch.xml`, you will see that loads two configuration files related to the `move_base` node: `move_base.yaml` and `fetch/move_base.yaml`.

Create a folder named `config` if it doesn't exist and copy these two files:
```bash
cd fetch_api/config
```

Both files are named `move_base.yaml`, so `roscp` will ask you which one to copy.
This command copies the main `move_base.yaml` (double check the numbers, which may be different for you):
```bash
> roscp fetch_navigation move_base.yaml .
You have chosen a non-unique filename, please pick one of the following:
1) /opt/ros/indigo/share/fetch_navigation/config/freight/move_base.yaml
2) /opt/ros/indigo/share/fetch_navigation/config/move_base.yaml
3) /opt/ros/indigo/share/fetch_navigation/config/fetch/move_base.yaml
#? 2
/opt/ros/indigo/share/fetch_navigation/config/move_base.yaml
```

This command copies the Fetch-specific configuration:
```bash
> roscp fetch_navigation move_base.yaml fetch_move_base.yaml
You have chosen a non-unique filename, please pick one of the following:
1) /opt/ros/indigo/share/fetch_navigation/config/freight/move_base.yaml
2) /opt/ros/indigo/share/fetch_navigation/config/move_base.yaml
3) /opt/ros/indigo/share/fetch_navigation/config/fetch/move_base.yaml
#? 3
/opt/ros/indigo/share/fetch_navigation/config/fetch/move_base.yaml
```

# Edit the launch files
To run the modified versions of the launch files, you will need to start navigation using this command from now on:
```bash
roslaunch fetch_api fetch_nav.launch
```

Also be sure to edit any launch files you have that include `fetch_nav.launch`:
```xml
<include file="$(find fetch_api)/launch/fetch_nav.launch">
```

However, you also need to edit your copy of `fetch_nav.launch` to use your copy of `move_base.launch.xml` and the two `move_base.yaml`s.

Edit `fetch_nav.launch` and replace:
```xml
<arg name="move_base_include" default="$(find fetch_navigation)/launch/include/move_base.launch.xml" />
```
with:
```xml
<arg name="move_base_include" default="$(find fetch_api)/launch/move_base.launch.xml" />
```

Edit `move_base.launch.xml` and replace:
```xml
<rosparam file="$(find fetch_navigation)/config/move_base.yaml" command="load" />
<rosparam file="$(find fetch_navigation)/config/$(arg name)/move_base.yaml" command="load" />
```
with (notice the three differences on the two lines):
```xml
<rosparam file="$(find fetch_api)/config/move_base.yaml" command="load" />
<rosparam file="$(find fetch_api)/config/fetch_move_base.yaml" command="load" />
```

# Setting the global max speed
If you just want the robot to move more slowly overall, you can change some navigation parameters, which are read once when fetch_nav.launch starts.

Edit `fetch_move_base.yaml` and set the `max_vel_x` parameter from 1.0 to something slower, like 0.3.
You can also limit its angular velocity (spinning speed) by changing `{min,max}_vel_theta`.
`max_vel_x` is in meters/second, `{min,max}_vel_theta` are in radians/second.

You can experiment with the tolerances in `move_base.yaml` (`yaw_goal_tolerance` and `xy_goal_tolerance`).
However, we have not tested how tight of tolerances the robot can actually achieve.

Refer to the course repository for a working example of the launch/config files and the edits we made.

# Dynamically adjusting the robot's speed
To dynamically adjust the robot's speed during navigation, we can rewrite the commanded velocities from move_base.

First, edit `launch/move_base.launch.xml`:
```diff
- <arg name="cmd_vel_topic" default="cmd_vel" />
+ <arg name="cmd_vel_topic" default="move_base_cmd_vel" />
```

Now, the `move_base` node will send velocity commands to the `move_base_cmd_vel` topic, which has no effect on the robot.
To get the robot moving again, we will republish the commands.

Create **fetch_api/scripts/navigation_controller.py**:
```py
#! /usr/bin/env python

"""Republishes navigation commands at user-controllable speeds.
"""

import geometry_msgs.msg
import numpy as np
import rospy
import std_msgs.msg

class NavigationController(object):
    def __init__(self, cmd_vel_pub):
        self._cmd_vel_pub = cmd_vel_pub
        self._max_speed = 1.0 # In meters/second

    def nav_cmd_vel_callback(self, twist):
        vec = np.array([twist.linear.x, twist.linear.y])
        commanded_speed = np.linalg.norm(vec)
        if commanded_speed > self._max_speed:
            squared_speed = commanded_speed * commanded_speed
            max_squared = self._max_speed * self._max_speed
            slow_factor = squared_speed / max_squared
            twist.linear.x /= slow_factor
            twist.linear.y /= slow_factor

        self._cmd_vel_pub.publish(twist)

    def max_speed_callback(self, msg):
        rospy.loginfo('Setting max speed to {}'.format(msg.data))
        self._max_speed = msg.data

def main():
    rospy.init_node('navigation_speed_controller')
    cmd_vel_pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    controller = NavigationController(cmd_vel_pub)
    sub = rospy.Subscriber('move_base_cmd_vel', geometry_msgs.msg.Twist, controller.nav_cmd_vel_callback)
    sub = rospy.Subscriber('navigation_controller/max_speed', std_msgs.msg.Float32, controller.max_speed_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
```

Make it executable:
```
chmod +x fetch_api/scripts/navigation_controller.py
```

This node subscribes to move_base's commanded velocities on `move_base_cmd_vel`.
In its callback, it checks if the commanded speed exceeds a threshold.
If so, it scales the speed such that the overall speed is at most the threshold.
It then publishes the modified message to `cmd_vel`.

To change the maximum speed, publish the new maximum speed to `navigation_controller/max_speed`, which is of type `std_msgs/Float32`.

You will need to run this node in conjunction with fetch_nav.launch.
In the course repo, we have modified `fetch_api/launch/fetch_nav.launch` to also start `navigation_controller` as well.