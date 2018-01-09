[Robot Web Tools](http://robotwebtools.org/tools.html) is a set of JavaScript libraries that lets web browsers communicate with ROS nodes.
It is not officially developed by ROS, but it is commonly used in the community.

# Web architecture
Unlike the C++ and Python implementations of ROS, the ROS JavaScript library (roslibjs) does not communicate directly with the master or with other nodes.

Instead, browsers communicate through an intermediate ROS node called `rosbridge_server` using a browser technology called [Websockets](https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API).
`rosbridge_server` communicates with browsers using Websockets and communicates with other ROS nodes using the normal mechanisms.
For example, if a web browser wants to subscribe to a topic, it notifies `rosbridge_server`, which then subscribes to that topic on the web browser's behalf and forwards messages back to the browser.

![ROS web architecture](https://cloud.githubusercontent.com/assets/1175286/24985441/faf63b5e-1fa8-11e7-8e2f-feb26488a67e.png)

The practical consequence of this is that you must run the `rosbridge_server` on the robot for your web apps to work:
```
sudo apt-get install ros-indigo-rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch
```

The web page itself is served from a normal web server like Apache.
Often, we will serve the website from the robot.
However, you can also serve the website from any computer (such as using cloud hosting) and even control multiple robots from the same website.

## Frontend vs. backend
Robot Web Tools will help us develop web-based frontends for the robot.
However, keep in mind that you need to implement the functionality on the robot.
We refer to this as the backend.
Typically, your backend will just be a ROS node that exposes topics or services that your web frontend can use.

# Robot web tools tutorials
Go to the [Robot Web Tools](http://robotwebtools.org/tools.html) website.
Notice that each of the three libraries, roslibjs, ros2djs, and ros3djs, have tutorials.
Tutorial 1 ([Writing an actionlib client](http://wiki.ros.org/roslibjs/Tutorials/ActionlibClient)) and Tutorial 3 ([Basic ROS functionality](http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality)) of the [roslibjs tutorials](http://wiki.ros.org/roslibjs/Tutorials) will be especially helpful.

# Create a web teleop package
In this lab, we will start developing the starter code for a web-based teleoperation interface.
In the starter project, we will have a web interface that allows us to:
- Read the current height of the torso
- Set a new height for the torso
- Moves the robot forward

This should give you an idea of how to implement the rest of the interface for your assignment.

![image](https://cloud.githubusercontent.com/assets/1175286/24991283/91d75fd8-1fce-11e7-8292-982a51123984.png)

Create the package `web_teleop`:
```
cd ~/catkin_ws/src/cse481c
catkin create pkg web_teleop --catkin-deps fetch_api joint_state_reader roscpp rospy
catkin build
source ~/.bashrc
```

Note that we are depending on the `fetch_api` and `joint_state_reader` that we implemented in previous labs.

# Implement the backend
The rest of this lab will focus on implementing the backend.
The next lab presents one way of implementing the frontend.

Below we show how to implement the backend for each requirement of this sample teleop interface:
1. Read the torso height
1. Set the torso height
1. Drive the robot forward

## 1. Read the torso height
In the previous tutorial, we created the `joint_state_reader`, which allows us to read the value of any joint.
If we publish the joint values to a topic, then a web browser will be able to subscribe to the topic and read the value.
So, we need to write a node that constantly publishes the torso height.

It is common practice for long-running backends to be placed in a folder called `nodes`, while fire-and-forget scripts like the demos we created in the previous labs are placed in a folder called `scripts`.
In this case, we are going to create a long-running backend:
```
cd web_teleop
mkdir nodes
touch nodes/joint_state_republisher.py
chmod +x nodes/joint_state_republisher.py
```

Here are the contents of `joint_state_republisher.py`:
```py
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from joint_state_reader import JointStateReader


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    torso_pub = rospy.Publisher('joint_state_republisher/torso_lift_joint',
                                Float64)
    reader = JointStateReader()
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # TODO: get torso joint value
        # TODO: publish torso joint value
        rate.sleep()


if __name__ == '__main__':
    main()
```

Note that once you figure out how to write this, you can easily extend this to get the values of other joints.

## 2. Set the torso height
To set the torso height, we will wrap our `fetch_api` code and provide a service to raise the torso.

First, create a `SetTorso.srv` file:
```
cd web_teleop
mkdir srv
cd srv
vim SetTorso.srv
```

SetTorso.srv:
```
float64 height
---
```

As you know from the ROS tutorials, there are a few more steps for your service message to be generated properly.
`CMakeLists.txt` contains detailed instructions on how to do this.

### CMakeLists.txt
In `CMakeLists.txt`, add `message_generation` to the `find_package` macro:
```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
)
```

Uncomment `add_service files`:
```cmake
add_service_files(
  FILES
  SetTorso.srv
)
```

Uncomment `generate_messages`:
```cmake
generate_messages(
# DEPENDENCIES
)
```

Scroll down and search for `catkin_package` and uncomment `CATKIN_DEPENDS`:
```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES web_teleop
CATKIN_DEPENDS message_runtime roscpp rospy
#  DEPENDS system_lib
)
```

### package.xml
In `package.xml`:
- Add `<build_depend>message_generation</build_depend>
- Add `<run_depend>message_runtime</run_depend>

Finally, run `catkin build` to build your service message.

### Implement the service
Create a node called `actuators_node.py`:
```
cd web_teleop
touch nodes/actuators_node.py
chmod +x nodes/actuators_nodes.py
```

Here is some starter code for you:
```py
#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        return SetTorsoResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    rospy.spin()


if __name__ == '__main__':
    main()
```

## 3. Drive the robot forward
The web browser will be able to publish commands to `/cmd_vel` directly, so we do not need to implement anything in the backend for this.

In fact, we could have reimplemented the code to aggregate the joint states and send a trajectory goal to the torso controller in JavaScript, in which case we don't need to write a backend at all.
However, as your application becomes more complex, you will eventually run into a case in which you can't implement something in JavaScript and need a ROS backend to help.

# Launching the backend
So far, we have created two backend nodes which need to be run.
We also know that `rosbridge_server` needs to be run.
This means that we need to open three terminal windows and remember the exact names of all the things we need to launch and run.

ROS supports `.launch` files, which specify a group of nodes or other launch files to run.
You can read all about it in the [roslaunch documentation](http://wiki.ros.org/roslaunch).

We will create a single launch file that launches everything we need to run our backend.
This means we only have one command to memorize.
Using .launch files also gives us more predictable startup and shutdown behavior.

It is convention to put .launch files in a folder named `launch`:
```
cd web_teleop
mkdir launch
touch launch/backend.launch
```

This is what should go in `backend.launch`:
```xml
<launch>
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" />
  <node pkg="web_teleop" type="joint_state_republisher.py" name="joint_state_republisher" />
  <node pkg="web_teleop" type="actuators_node.py" name="web_teleop_actuators" />
</launch>
```

Now you should be able to launch your backend with:
```
roslaunch web_teleop backend.launch
```