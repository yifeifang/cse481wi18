# ROS tutorials

First, you will need to do the basic [ROS tutorials](https://wiki.ros.org/ROS/Tutorials).
We have a few twists on it for you to follow:

Important notes:
- We are using the catkin build system, not rosbuild
- We are using ROS Indigo, not Kinetic

# `catkin_make` vs. `catkin`
There are two tools for using catkin: `catkin_make` and `catkin` (aka [catkin tools](http://catkin-tools.readthedocs.io/en/latest/)).
The ROS tutorials assume you use `catkin_make`, but we recommend using `catkin`.
With `catkin`, it is easier to visualize concurrent builds, and, best of all, you do not have to be in the root directory of your workspace to run `catkin build` (you do if you use `catkin_make`).

With this in mind, do Tutorials 2-18 with the following tweaks:
- **Tutorial 3**: Instead of running this command:
  ```diff
  - catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
  ```
  Run this:
  ```diff
  + catkin create pkg beginner_tutorials --catkin-deps roscpp rospy std_msgs
  ```

  And from now on, never run this:
  ```diff
  - catkin_make
  ```
  Instead, whenever you see `catkin_make`, run this instead:
  ```diff
  + catkin build
  ```
- **Tutorial 4**: Again, remember to replace `catkin_make` with `catkin build`
- **Tutorial 10**: Replace `catkin_make install` with `catkin build --install`
- **Tutorial 11**: Replace `catkin_make` with `catkin build`
- **Tutorial 12**: Replace `catkin_make` with `catkin build`
- **Tutorial 14**: Replace `catkin_make` with `catkin build`
- **Tutorial 15**: Replace `catkin_make` with `catkin build`