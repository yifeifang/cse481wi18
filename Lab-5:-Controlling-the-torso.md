The robot's torso can rise 0.4 meters.

# actionlib
As in Lab 4, you must be familiar with [actionlib](http://wiki.ros.org/actionlib) and the [actionlib](http://wiki.ros.org/actionlib/Tutorials) tutorials before starting.

# Trajectories
If you read the Fetch docs on the [arm and torso](http://docs.fetchrobotics.com/api_overview.html#arm-and-torso), you will see that to control the torso, you must send a *trajectory* as opposed to a position to lift the torso to.

A trajectory is simply a list of (position, time) pairs, which specifies a motion with multiple in-between points.
A trajectory can also specify the desired velocity and acceleration at each point of time, which can result in smoother motions.
Generally, the robot's controllers will automatically interpolate trajectories for you, so you only need to send the endpoint.
This works well for the torso, but may not work well for complicated mechanisms like arms.

# Torso trajectories
The [robot hardware overview](http://docs.fetchrobotics.com/robot_hardware.html#forces-and-torques) in the Fetch docs provides position, rotation, force, and velocity limits for all of the robot's joints.
We can see that the torso's position limit is 0.4 m, and its velocity limit is 0.1 m/s.
This means that from any given torso position, it should take no longer than 4 seconds to reach any other torso position at maximum velocity.
However, to account for gradually speeding up and slowing down, as well as to avoid exceeding the velocity limit, let's assume that time is 5 seconds instead.

As a result, all you have to do to control the torso is to send a trajectory with a single point: the end point.
The end point will be the desired torso height, and the desired time to reach that height will be 5 seconds.

# Write the torso class
As in the previous labs, we have implemented a skeleton of a `Torso` class for you in `fetch_api/src/fetch_api/torso.py`.
Implement the `set_height` method.

**Hints:**
* What is the name of the action server to control just the torso?
* What's the best way to find a message definition? Google. Just search for, e.g., "trajectory_msgs/JointTrajectory" and click on the first result. Be sure, however, to change `kinetic` in the URL to `indigo`, as some message definitions change between ROS versions.
* The `joint_names` field of a `JointTrajectory` msg is actually a list of strings. To control the torso, you only need to add one item to the list, since you are only controlling one joint.
* You can find a list of joints in the Fetch docs in the [hardware overview](http://docs.fetchrobotics.com/robot_hardware.html#joint-limits-and-types) section. Which joint corresponds to the torso?
* If, in a msg definition, you see something like `JointTrajectoryPoint`, that means that `JointTrajectoryPoint` is in the same package as the msg you are looking at (i.e., its full name is `trajectory_msgs/JointTrajectoryPoint`).
* Notice that JointTrajectoryPoint allows you to specify positions, velocities, accelerations, and efforts. You only need to specify a position in this case.
* The `positions` field of JointTrajectoryPoint is a list of floats. The values of the list correspond to the list of joints. Since we are only controlling one joint, this list only needs to have one value in it.
* Set the `time_from_start` to 5 seconds. A `duration` type in a ROS msg corresponds to `rospy.Duration` in Python (see the [msg documentation](http://wiki.ros.org/msg#Message_Description_Specification)).
* As discussed earlier, our "trajectory" only has one JointTrajectoryPoint: the end point.

# Finish the demo
As in the gripper lab, we have partially implemented a demo file for you.
However, this time you must import the right module, instantiate the Torso object, and call its `set_height` method.

You should now be able to run the following and see the robot's torso move up and down:
```
rosrun applications torso_demo.py 0.1
rosrun applications torso_demo.py 0.4
rosrun applications torso_demo.py 0.0
```