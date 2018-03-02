The labs explain how we can plan a path for the gripper to get to a goal pose using the MoveIt motion planning framework.
However, you may have noticed that sometimes the gripper moves in an arc or some other unusual path.

This is because the mapping between the robot's *joint space* (a 7-dimensional vector of joint angles along the arm, also called the *configuration space*) and the *task space* (the XYZ position and orientation of the gripper) is not very straightforward.
The motion planning algorithms we use generally look for the shortest path in *joint space* that connects the current arm pose to the goal pose, which is not necessarily the shortest path in *task space*.

Often, you will want to plan straight-line motions (sometimes called *snap planning*).
You should only use this method if you know that a particular motion is collision-free when moving in a straight line.
In our implementation, the arm will stop if it is about to collide with a known obstacle.

# Analyze the feasibility
Before you commit to executing a straight-line motion, you may want to conduct a simple test to see if doing so is a reasonable idea:

- Launch Gazebo, MoveIt, and RViz.
- Open the [MotionPlanning display in RViz](Tutorial%3A-MoveIt-RViz-plugin)
- Move the arm to what you think the starting pose will be
- Drag the gripper to the end pose

If you see the arm flip around wildly, this indicates that the arm is passing through a *singularity.*
A singularity is a configuration where the end-effector loses its ability to smoothly move in one or more directions.
In order move in the restricted direction, the robot needs to "jump" to a different configuration, resulting in infinitely fast motion for some joints.

[![image](https://media.giphy.com/media/5QYiYtvoGYIY4xaRQG/giphy.gif)]()

If, on the other hand, you see the arm moving smoothly through the motion, this means that a straight-line path is probably feasible:

[![image](https://media.giphy.com/media/1msK2e5i1CRlOqPluA/giphy.gif)]()

# Update arm.py
We have updated [`fetch_api/src/fetch_api/arm.py`](https://github.com/cse481wi18/cse481wi18/blob/indigo-devel/fetch_api/src/fetch_api/arm.py#L239) in the course repository with a new method:
```py
def straight_move_to_pose(self, group, pose_stamped, ee_step=0.025, jump_threshold=2.0)
```

To implement this, we use a different MoveIt API than we used for the other arm methods.

# Demo
[`applications/scripts/straight_gripper_motion_demo.py`](https://github.com/cse481wi18/cse481wi18/blob/indigo-devel/applications/scripts/straight_gripper_motion_demo.py) illustrates how to use this new API:
```
roslaunch fetch_gazebo playground.launch
roslaunch fetch_api move_group.launch
rosrun applications torso_demo 0.4
rosrun applications straight_gripper_motion_demo.py
```

A minimal code snippet showing how to use `straight_move_to_pose` is shown below:
```py
import moveit_commander
import sys

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    moveit_robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    def on_shutdown():
        group.stop()
        moveit_commander.roscpp_shutdown()

    rospy.on_shutdown(on_shutdown)

    arm = fetch_api.Arm()
    error = arm.straight_move_to_pose(group, pose_stamped)
    if error is not None:
        rospy.logerr(error)

    moveit_commander.roscpp_shutdown()
```

This will attempt to plan a straight-line motion to the goal pose.

If you are unable to plan a path, check for the following:
- Are you passing through any singularities?
- Is the path clear of obstacles?
- If neither of the above are true, try setting jump_threshold higher (its default value is 2.0, so try small changes like 2.5, 3.0, etc.)