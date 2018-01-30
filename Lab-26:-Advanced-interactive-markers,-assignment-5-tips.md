In your next assignment, you will have to create an interactive marker of a Fetch gripper.
The marker needs to have a clickable menu to go to the gripper pose, open the gripper, or close the gripper.
You also need to change the color of the gripper depending on whether an IK solution was found for that pose.
This lab goes over how to accomplish some of these tasks.

# IKFast
Make sure you have completed the previous lab on getting IKFast working.
Otherwise, your feedback callbacks will take a long time to run.

# General structure
As usual, we recommend wrapping your code in a class:
```py
class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # gripper_im = InteractiveMarker() ...
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    ...
    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()
```

# How to make a gripper marker
You can make a marker out of a mesh.
See [rviz/DisplayTypes/Mesh](http://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D).

Here are the package URIs for the gripper meshes:
```py
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
```

You will need to create 3 markers: one for the gripper and two for the fingertips.
These markers will be added to a single `InteractiveMarkerControl`, which in turn is added to your `InteractiveMarker`.
You should create a function that, given a `PoseStamped`, returns either an `InteractiveMarker` or a list of 3 Markers.
See what the marker looks like when you place it at 0, 0, 0, in the `base_link` frame.
You can use the grid lines to get a sense of how the meshes are laid out (each grid square in RViz is 1 meter by 1 meter).
You will need to make some adjustments to the fingertip positions.

# What is the end-effector link?
To our eyes, `gripper_link` is the most intuitive end-effector link.
However, MoveIt is actually configured with `wrist_roll_link` as the end-effector.
You can find the offset between `wrist_roll_link` and `gripper_link` using:
```
rosrun tf tf_echo wrist_roll_link gripper_link
```

Initially, your gripper marker, when placed at (0, 0, 0) will look like this:

![image](https://cloud.githubusercontent.com/assets/1175286/25469624/47aa90a6-2ad2-11e7-8908-84b0e07e7701.png)

This is because the meshes are defined in the `gripper_link` frame.
However, you actually want the marker to be centered on the `wrist_roll_link`, like below.
Use the offset above to correct this.

![image](https://cloud.githubusercontent.com/assets/1175286/25469637/63b20d74-2ad2-11e7-9a38-b2defad7c8c6.png)

# Creating 6 DOF controls
Follow the [Interactive Markers: Basic Controls tutorial](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls#Simple_6-DOF_control).
That tutorial is written in C++ and does some unnecessary stuff for illustrative purposes, so try to extract the essence of the tutorial instead of following it directly.
To create the arrows and rings around the marker, you will need to add 6 different controls to your interactive marker, as shown in the tutorial.
Once you are done, the arrows and rings will probably be ginormous, so scale them down using the `scale` field of the `InteractiveMarker`.

You will need to add 6 DOF markers to many grippers, so we recommend creating a function that returns a list of `InteractiveMarkerControl`.
You can then append these controls to an interactive marker:
```py
controls = make_6dof_controls()
interactive_marker.controls.extend(controls)
```

If you are following the C++ tutorial and for some reason, only one of the 6 DOF controls is showing up, then it might be because you are forgetting to use `copy.deepcopy`:
```py
control = InteractiveMarkerControl()
control.name = 'move_x'
controls.append(control)
control.name = 'rotate_x'
controls.append(control)
# Oops, control is the same object in both cases. You need to make a copy with copy.deepcopy(control).
```

# Menu items
An [`InteractiveMarker` msg](http://docs.ros.org/indigo/api/visualization_msgs/html/msg/InteractiveMarker.html) contains a list of `MenuEntry` msgs.
According to the `MenuEntry` documentation, you just assign a non-zero ID to each menu entry, and set its `command_type` to `FEEDBACK`.
Then, in your feedback callback, you can check if the `event_type` is `MENU_SELECT` and if so, use `menu_entry_id` to figure out which menu item was clicked.

Finally, you will need to set the `interaction_mode` of the control that holds your gripper markers to `MENU`.

# Handling drag events
If your gripper is disappearing when you drag the 6 DOF controls, you need to set `always_visible` on the control that holds the gripper.

To check IK on drag events, check if the `event_type` is `POSE_UPDATE` in your feedback callback.

# Changing the gripper color
This is easy.
Just get your `InteractiveMarker`, iterate through the list of markers that comprise your gripper visualization, and change their colors individually.
Then, reinsert your `InteractiveMarker` to your interactive marker server and call `applyChanges()`.

# Running on the real robot

Make sure that you can execute the assignment tasks in simulation with the blue cube before trying it on the real robot.

The robot's wireless connection is very slow for some reason.
While we address this issue, you will need to take the following steps:
1. Update your code
1. Add assignment 5 obstacles file to your repository
1. Ship your code to the robot
1. Run MoveIt on the robot
1. Add the obstacles and position the robot
1. Run your gripper teleop node
1. Execute actions carefully
1. Run-stop the robot

## 1. Update your code
When you create your interactive marker servers, set the `q_size` arg to a small number like 2:
```py
im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
auto_pick_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
```

## 2. Assignment 5 obstacles file
You **must** run this code to add the bottom and top shelves of the classroom into the PlanningScene.
Add this code to your repository in `applications/scripts/a5_obstacles.py`.
Mark it as executable and push it to Github.

```py
#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print 'Usage: rosrun applications a5_obstacles.py'
    print 'Drive the robot until the PlanningScene lines up with the point cloud.'


def main():
    rospy.init_node('a5_obstacles')
    wait_for_time()

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('floor')
    planning_scene.addBox('floor', 2, 2, 0.01, 0, 0, 0.01/2)
    planning_scene.addBox('table', 0.5, 1, 0.72, 1, 0, 0.72/2)

    rospy.sleep(2)


if __name__ == '__main__':
    main()
```

## 3. Ship your code to the robot
When we work on perception, your code will only run reasonably fast if you run it directly on the robot.
We will practice shipping your code to the robot.
To do this, log into `astro` with your team's username:

```
ssh teamN@astro
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd ~/catkin_ws/src
git clone https://github.com/cse481wi18/teamN.git cse481c
catkin build
source ~/.bashrc
```

Make sure that your IKFast plugin is also in the repository and that you've built it.

You may also generate an SSH key pair on the robot and add it to Github, as in Lab 1.

## 4. Run MoveIt on the robot

While SSH'ed into the robot, run the code below.
You might want to open a tmux window first, so that you don't have to keep opening terminal windows to SSH into astro.
```
roslaunch fetch_api move_group.launch
```

Make sure you don't get any red error messages while launching MoveIt.

## 5. Add the obstacles and position the robot
On your lab computer, run RViz.
```
setrobot astro
rosrun rviz rviz
```

- You may notice RViz being slower than normal.
- Add a PointCloud2 display and select the `/head_camera/depth_downsample/points` topic.
- You may want to set the ColorTransformer to AxisColor.
- Also add a PlanningScene display, a Trajectory display, and your interactive marker servers.

While SSH'ed into the robot, add the obstacles:
```
rosrun applications a5_obstacles.py
```

The obstacles are specified in the `base_link` frame.
Carefully drive the robot around [using the joystick](http://docs.fetchrobotics.com/teleop.html) until the obstacles match up with the point cloud.
It may help to change the color of the PlanningScene (e.g., to dark red) to help see the point cloud.
It might also help to move the head around to see the top and bottom shelves.
Finally, you should set the robot to the maximum height to improve your chances of getting good motion plans.

You should see something like this:
![image](https://cloud.githubusercontent.com/assets/1175286/25516110/6ab1a0d2-2b9d-11e7-8984-568028f2dcef.png)

Do not continue unless you see the obstacles match up with the point cloud.

## 6. Run your gripper teleop node
Run your gripper teleop node on the robot.
By "on the robot" we mean inside your SSH terminal.

```
rosrun applications gripper_teleop.py # Or whatever it's named
```

## 7. Execute actions carefully
When commanding the arm to move, be very careful.
You should always have someone on the left side of the robot, ready to press the robot's runstop.
Make sure that person knows when you are about to execute a motion plan on the robot.

Do not directly make a motion plan to a pre-grasp pose.
Even with the obstacles in the planning scene, various imprecisions in the system might lead to a collision anyway.
Instead, guide the arm to a clear area above the obstacle (but below the top shelves) using a sequence of movements.

Check the visualization from a multitude of angles, and make sure the gripper and the fingertips are not intersecting with the point cloud.
It often helps to grasp the object from the side and at a slight vertical angle.
That way, you keep the bulky wrist away from obstacles, only putting the fingertips at risk.
Grasping from the side also allows you to maintain vision of the object in RViz.

## 8. Run-stop the robot
When you are done, runstop the robot.
This will prevent you or other teams from accidentally running motion plans on the real robot.

You should also close any terminal windows in which you ran `setrobot astro`, such as your RViz terminal.
It often helps to isolate any terminal windows in which you run `setrobot astro` in a single tmux session, so that you can easily prevent yourself from accidentally running code on the real robot.

## If the arm stops working
If the arm stops working, SSH into the robot and [reset the breakers](http://docs.fetchrobotics.com/api_overview.html#resetting-breakers).

# Assignment videos
Here are some videos of what your assignment might look like.

Gripper teleop (simulation):

[![image](http://i3.ytimg.com/vi/fmbaHcKUPgU/hqdefault.jpg)](https://www.youtube.com/watch?v=fmbaHcKUPgU)

Gripper teleop (real robot):

[![image](http://i3.ytimg.com/vi/n535MInCH6A/hqdefault.jpg)](https://www.youtube.com/watch?v=n535MInCH6A)

Gripper teleop with automated pick sequence (simulation):

[![image](http://i3.ytimg.com/vi/Scc5ph2ZA0s/hqdefault.jpg)](https://www.youtube.com/watch?v=Scc5ph2ZA0s)

Gripper teleop with automated pick sequence (real robot):
[![image](http://i3.ytimg.com/vi/ROa3rsWNPGg/hqdefault.jpg)](https://www.youtube.com/watch?v=ROa3rsWNPGg)