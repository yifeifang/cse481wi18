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

![image](https://media.giphy.com/media/5QYiYtvoGYIY4xaRQG/giphy.gif)

If, on the other hand, you see the arm moving smoothly through the motion, this means that a straight-line path is probably feasible:

![image](https://media.giphy.com/media/1msK2e5i1CRlOqPluA/giphy.gif)

# Update arm.py
We have updated `fetch_api/src/fetch_api/arm.py` in the course repository with a new method:
```py
```