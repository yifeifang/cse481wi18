In [Lab 7](https://github.com/cse481wi18/cse481wi18/wiki/Lab-7%3A-Controlling-the-arm), we saw how to control the robot's arm by setting the joint angles of the arm.
This is the API that the Fetch provides.
However, it's easier to think about arm movements in terms of X/Y/Z position of the gripper (a.k.a., Cartesian space).

# Overview of some concepts
## Inverse kinematics
Given the joint angles of the arm, it's easy to compute where the gripper (the *end-effector*) will be, since we know how long the arm links are and how they are connected.
This is called the *forward kinematics* of the arm.
The *inverse kinematics* (IK) problem is, given the desired end-effector pose, to find out what the joint angles should be for the end-effector to achieve that pose.
This is a harder problem.
Sometimes, there are multiple arm configurations that result in the same end-effector pose, while other times, the pose is not reachable.
The way to solve the IK problem depends on what kind of arm you have.
For some arms (including that of the Fetch), there is a closed-form solution that makes it relatively easy to solve IK, but this is not true for all arms.

When you use command the robot's gripper to a particular pose, the robot can simply compute the IK solution to figure out the joint angles, and then move the robot's arm using the joint angles.
However, you need to be mindful of the fact that IK solutions sometimes result in weird arm motions that might collide with the environment or flip the gripper upside down (bad if you're holding a tray or a glass of water).
So, sometimes, you want to have complete control over the joint angles.

## Motion planning
To avoid obstacles or ensure that the gripper stays in a particular orientation, you will want to do *motion planning*.
Motion planning generates an arm trajectory for you given the desired goal and any constraints you have (such as the need to avoid obstacles, stay within a workspace, or keep the gripper upright).

## MoveIt
The primary motion planning framework for ROS is called [MoveIt](http://moveit.ros.org/).
You can find more information from:
- The official [MoveIt tutorials](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html) from the MoveIt website.
- The [Fetch manipulation tutorial](http://docs.fetchrobotics.com/manipulation.html)

### `MoveGroup`
MoveIt provides an action, called [MoveGroup](http://docs.ros.org/indigo/api/moveit_msgs/html/action/MoveGroup.html) that actually triggers the motion planning and execution given a goal.
You run the action server as part of the MoveIt backend, `move_group.launch`.
There are many options to set as part of the MoveGroup action, so MoveIt provides Python and C++ classes that wrap the action client and provide convenience methods for you.

In case you are wondering, a "group" is MoveIt's term for a set of joints to plan for.
In the case of the Fetch, there are three groups that have been pre-configured for you: *arm*, *arm_with_torso*, and *gripper*.
The difference between the *arm* and *arm_with_torso* groups is that the *arm_with_torso* will plan to move the torso along with the arm, while the *arm* group can only plan to move the arm.
In this lab, we will use the *arm* group, but using *arm_with_torso* is perfectly reasonable.

### Our MoveIt interface
MoveIt can be used with both Python and C++ interfaces.
We will be using Python.
When we get to the perception labs in a couple of weeks, at least one or two members of your group will need to be comfortable with C++.

There are two Python interfaces for MoveIt: `moveit_commander` and `moveit_python`.
`moveit_commander` is the "official" one and is used in the MoveIt tutorials.
It works by using a wrapper around the C++ API.

`moveit_python` was written by a Fetch Robotics employee and is used in the Fetch manipulation tutorial.
It is written in pure Python (i.e., it does not use a C++ wrapper) and supports a subset of the functionality provided by `moveit_commander`.

Unfortunately, neither Python interface supports all the features that we will need in this class.
Instead, you will use a custom interface written for this class.
However, the same basic ideas will transfer to whatever code you use in the future.

Both existing Python MoveIt interfaces wrap an ActionClient for the [MoveGroup Action](http://docs.ros.org/indigo/api/moveit_msgs/html/action/MoveGroup.html).
However, `moveit_commander` hides the action client from you and always waits for infinite time for an action execution to complete.
This is problematic, as we'll see.
`moveit_python` gives you access to the action client, but it lacks the full feature set of the MoveIt system.

Our MoveIt interface is the best of both worlds.
All our interface does is provide an easy way to generate MoveGroup action goals, which are quite complicated.
You write the action client yourself and send one of our generated goals.

# Sending Cartesian goals for the gripper
In this lab, we will get started with the very basics of MoveIt, following the [Fetch manipulation tutorial](http://docs.fetchrobotics.com/manipulation.html#simple-moveit-wave-example), but using our MoveIt interface.

You can find our interface in the course repository in [fetch_api/src/fetch_api/moveit_goal_builder.py](https://github.com/cse481wi18/cse481wi18/blob/indigo-devel/fetch_api/src/fetch_api/moveit_goal_builder.py).
Copy this code to your own repository.
You will also need to add  `from .moveit_goal_builder import MoveItGoalBuilder` to `__init__.py`.

**Method signature**

In `fetch_api/arm.py`, add a method to `Arm` called `move_to_pose`:
```py
def move_to_pose(self, pose_stamped):
    """Moves the end-effector to a pose, using motion planning.

    Args:
        pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

    Returns:
        string describing the error if an error occurred, else None.
    """
    pass
```

**Imports**

To implement `move_to_pose`, you will first need to import:
```py
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction                                  
```

**Initialization**

Create a `MoveGroupAction` action client in your `__init__` method.
The name of the action server is `move_group`.

**Sending the goal**

To create a goal, call:
```
goal_builder = MoveItGoalBuilder()
goal_builder.set_pose_goal(pose_stamped)
goal = goal_builder.build()
```

Next, fill out `move_to_pose` based on the manipulation tutorial linked above.
- Look at the definition of the `MoveGroup` action (linked above). You should see that the `MoveGroupResult` section contains a field called `error_code`.
- Look up the errors that can occur.
- Your method should return `None` on success, or an error string if the error code of the action result is not `SUCCESS`.
- We provide a utility function for mapping an error code to a string below.
- For didactic purposes, send your goal and wait forever for a result:
```py
self._move_group_client.send_goal(goal)
self._move_group_client.wait_for_result()
```

**Cancel method**

Finally, for safety purposes, add a `cancel_all_goals()` method to your `Arm` class, which will be called in your demo:
```py
def cancel_all_goals(self):
    self._client.cancel_all_goals() # Your action client from Lab 7
    self._move_group_client.cancel_all_goals() # From this lab
```

**Converting error codes to strings**

This is a helpful utility function:
```py
def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'                                      
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'
```

# Wave demo
Write a demo called `cart_arm_demo.py` that repeatedly moves the gripper between these two poses:
```py
pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
ps1 = PoseStamped()
ps1.header.frame_id = 'base_link'
ps1.pose = pose1
ps2 = PoseStamped()
ps2.header.frame_id = 'base_link'
ps2.pose = pose2
gripper_poses = [ps1, ps2]
```

Your demo should use your new arm method and check the error code:
```py
error = arm.move_to_pose(pose)
if error is not None:
    rospy.logerr(error)
```

In your `main` function, be sure to cancel all goals on shutdown.
This is a safety feature that ensures that if you accidentally run the code on the real robot, you can stop the arm from moving by just killing your node.
Make sure the register the shutdown handler **BEFORE** entering any infinite loops.
```py
def main():
   # ... init ...
   arm = ...
   def shutdown():
       arm.cancel_all_goals()
   rospy.on_shutdown(shutdown)
   
   # Move the arm
```

# Wave demo attempt 1
With `roscore` and Gazebo running, launch the version of MoveIt pre-configured for the Fetch:
```
roslaunch fetch_moveit_config move_group.launch
```

Wait for this message to appear before continue:
```
All is well! Everyone is happy! You can start planning now!
```

Now run your node and wait 10 seconds:
```
rosrun applications cart_arm_demo.py
```

If you implemented your error handling correctly, you should see the following output:
```
/cart_arm_demo main:38: PLANNING_FAILED
/cart_arm_demo main:38: PLANNING_FAILED
```

Take a look also at the terminal where you launched MoveIt.
You should see output like this:
```
/move_group :224: LBKPIECE1: Created 1 (1 start + 0 goal) states in 1 cells (1 start (1 on boundary) + 0 goal (0 on boundary))
/move_group :117: No solution found after 10.002214 seconds
/move_group :529: Unable to solve the planning problem
/move_group MoveGroupMoveAction::executeMoveCallback_PlanAndExecute:95: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
/move_group PlanExecution::planAndExecuteHelper:176: Planning attempt 1 of at most 1
/move_group :243: No optimization objective specified, defaulting to PathLengthOptimizationObjective
/move_group :113: LBKPIECE1: Starting planning with 1 states already in datastructure
/move_group :141: LBKPIECE1: Unable to sample any valid states for goal tree
```

This indicates to you that MoveIt was unable to find a path to move the arm.
You can also see that the default planning time (before MoveIt gives up) is 10 seconds.
The reason why this fails is because the gripper goal poses are quite high up (about 1.8 meters above the ground).
So, you need to raise the torso before running your demo:
```
rosrun applications torso_demo.py 0.4
```

In this case, it's also perfectly reasonable to use the `arm_with_torso` group so MoveIt can move the torso up for you automatically.
But, by not doing so, we were able to test our error code handling.
And, depending on your application, you may want to keep the torso height under your control.

Run your demo again and you should see the robot wave its gripper.
However, if you watch it closely, you may notice that it occasionally takes weird paths that don't look like waves at all, especially in the beginning.

# Wave demo attempt 2
You may find that your demo gets stuck, especially if you wait forever for your action client to return a result, as we asked:
```py
self._move_group_client.wait_for_result() # Waits forever for a result.
```

After waving one or two times, it may just sit there forever, not even returning an error message.
There are two issues here:
1. There is a problem running MoveIt
1. Your program is not detecting this error and is just silently waiting for an infinite amount of time

We will focus first on problem 2.
Problem 2 illustrates an important fact.
Even though we have implemented error handling, sometimes there can be issues out of your control, especially with MoveIt.
We never want to have an error condition like this occur without somehow detecting it.
However, in this case, the program is just silently hanging.

*Almost nothing in a robot system should be allowed to wait for an infinite time. Put timeouts on everything and fail fast.*

```py
client.send_goal(goal)
client.wait_for_result(rospy.Duration(10))
result = client.get_result()
```

Or alternatively,
```py
client.send_goal_and_wait(goal, rospy.Duration(10))
result = client.get_result()
```

# The MoveIt problem
Look at the output of `move_group.launch`, when it was getting stuck.
You might see something like:

```
/move_group TrajectoryExecutionManager::validate:974: 
Invalid Trajectory: start point deviates from current robot state more than 0.01
joint 'upperarm_roll_joint': expected: -0.640106, current: -0.62881
```

This is an internal error where MoveIt is unable to run a trajectory because it appears the robot's arm is not in the same place as where it should be at the start of the new trajectory.
This happens because MoveIt records the "start" position of the second trajectory before the arm has settled down from the first motion.

If you run your demo now, you will see that it still gets stuck while waving, but after 10 seconds, it gives up and sends a new goal.

This gives us a robust retry behavior, but we haven't addressed the underlying issue we just discussed.
To give the arm some time to settle down, add a `rospy.sleep(1)` in between each pose.