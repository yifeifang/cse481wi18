In assignment 6, you will be developing a miniature version of a *programming by demonstration* (PbD) system.
This system captures the essence of how most industrial robots are programmed today.
In PbD, you program the robot to do a task by moving its arm to a sequence of poses.
The system saves the sequence of poses as an action and can execute the action again at a later time.

An extra feature of the system you will be developing is that instead of saving poses in the base frame, your system will provide the option to specify poses as offsets relative to an AR tag.
By attaching an AR tag to an object, the robot can locate the object and execute an action like grasping the object.

You should be able to use your PbD system to develop your final projects, so you will want to invest some time and do a good job with this assignment.

# Example workflow
## Creating an action
1. The robot starts at the table. The table has an object with a tag on it, and a tag near the edge.
1. The user presses the "Create program" button on an interface.
1. The robot's arm relaxes.
1. The user moves the arm to a pre-grasp pose and presses a "Save pose" button.
1. The interface asks if the pose should be relative to the base frame or to a tag.
1. The user says it should be relative to tag 1.
1. The user presses the "Open gripper" button.
1. The user saves another pose with the gripper in the grasp pose and saves it relative to the tag 1.
1. The user presses the "Close gripper" button.
1. The user saves another pose at the dropoff location and saves it relative to tag 2.
1. The user presses "Open gripper."
1. The user moves the arm to a final resting position and saves a pose relative to the base frame.
1. The user presses the "Save program" button, and the program is saved to a file. You will need to create a data structure to represent the program.

## Executing a program
1. A separate application loads the program from disk and executes it.
1. The user rearranges the scene slightly and runs the program again.

# Relaxing / freezing the arm
Whenever you move the robot's arm using MoveIt, you will notice that the arm freezes and cannot be moved by hand.
To enable users to program the robot by moving the arm, you will need to relax the robot's arm.
To do this, you will need to stop the robot's arm controller.
However, to send MoveIt commands to the arm, you will need to start the arm controller.
In short, before you create a program, stop the arm controller, and before you execute a program, start the arm controller.

To start or stop the arm controller, we will use an undocumented feature of the Fetch robot.
The Fetch robot API exposes an action named `/query_controller_states` of type [robot_controllers_msgs/QueryControllerStates](http://docs.ros.org/indigo/api/robot_controllers_msgs/html/action/QueryControllerStates.html).
You will need to create an action client to use this action.

To stop the arm controller (relax):
```py
goal = QueryControllerStatesGoal()
state = ControllerState()
state.name = 'arm_controller/follow_joint_trajectory'
state.state = ControllerState.STOPPED
goal.updates.append(state)
self._controller_client.send_goal(goal)
self._controller_client.wait_for_result()
```

To start the arm controller (before sending any MoveIt commands):
```py
goal = QueryControllerStatesGoal()
state = ControllerState()
state.name = 'arm_controller/follow_joint_trajectory'
state.state = ControllerState.RUNNING
goal.updates.append(state)
self._controller_client.send_goal(goal)
self._controller_client.wait_for_result()
```

# Transform arithmetic
You may want to revisit [[Lab 24: Transform arithmetic]].
Let's say you have <sup>B</sup>T<sub>tag1, t1</sub> and <sup>B</sup>T<sub>wrist, t1</sub> at t=1.
At runtime (t=2), the tag `tag1` is in a slightly different pose, given as <sup>B</sup>T<sub>tag1, t2</sub>.
Where should the wrist go at time=2?

Hints:
- The programmed offset between the wrist and tag is <sup>tag1, t1</sup>T<sub>wrist, t1</sub>
- <sup>A</sup>T<sub>B</sub> is the matrix inverse of <sup>B</sup>T<sub>A</sub>
- You want to compute <sup>B</sup>T<sub>wrist, t2</sub>

So, you will need to chain the transformations together and invert some of the matrices.

# Developing in simulation
You will want to test your system in simulation first, because otherwise the robot will be too busy.

## Perception
You should save a few point clouds of the table with AR tags on it and hallucinate them in simulation as explained in the previous labs.

## Moving the arm
In your system, users should physically move the robot's arm.
However, you can't do this in simulation.
Instead, you can just teleop the arm using the teleop applications we developed in previous labs.
Note that while on the real robot, you want to relax the arm, you always need to switch the arm controller on when sending MoveIt commands (e.g., when using teleop).
As a result, when you are in simulation, you should not relax the arm when programming.

To know if you are in simulation, you can check if the value of `use_sim_time` is set on the [ROS parameter server](http://wiki.ros.org/rospy/Overview/Parameter%20Server).

## Summary table for the arm controller
Phase | Real world or simulation | Arm controller enabled or disabled
--- | --- | ---
Program creation | Simulation | Enabled
Program creation | Real world | Disabled
Program execution | Simulation | Enabled
Program execution | Real world | Enabled

## Robot calendar
Please use the [Fetch robot calendar](https://calendar.google.com/calendar/embed?mode=week&src=cs.washington.edu_u990n37gkbs31e810jctf0ial8%40group.calendar.google.com&ctz=America/Los_Angeles) to book time on the robot, in blocks of no more than 90 minutes.