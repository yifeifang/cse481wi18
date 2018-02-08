While working in simulation, you may need to move the arm.
You can use your gripper teleop app from assignment 5 to do so.
Another useful tool is the MoveIt RViz plugin.

# Launch everything
To use the MoveIt RViz plugin, you will need to be running Gazebo, MoveIt, and RViz

# Open the MotionPlanning display
- Click "Add" in the displays panel.
- Select the "MotionPlanning" display.

![image](https://user-images.githubusercontent.com/1175286/36002604-9b3dd3b4-0cdf-11e8-9477-eacbbe231a57.png)

# Move the arm to a pose
Drag the gripper marker and 6 DOF controls to set a goal pose for the gripper.
The gripper will turn green if it is valid and red otherwise.
The orange arm shows the inverse kinematics solution for the goal pose.

To actually move the arm, click on the planning tab, and click "Plan and Execute."

![image](https://user-images.githubusercontent.com/1175286/36002752-2635c24c-0ce0-11e8-881a-a9bd5a55a20c.png)

If MoveIt can find a plan, the robot's arm will move as shown.

