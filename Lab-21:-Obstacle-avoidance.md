In the previous labs, when we used motion planning, that was in the absence of any obstacles.
Including obstacles in the scene makes the motion planning problem harder, but luckily, MoveIt can automatically plan around them, as long as you tell it where the obstacles are.

# `PlanningSceneInterface`
The MoveGroup action will plan a path that is free of self-collisions, but the arm could still collide with other parts of the environment.
`PlanningSceneInterface` MoveIt's representation of the world.
You use this to specify what the robot should not collide with.
To do this, you "add" primitive shapes or meshes representing the workspace to the planning scene.

MoveIt also has built-in capability to automatically infer the planning scene from sensor data.
However, this is most likely will be very slow, so we recommend using shapes and meshes for now.

# Create a demo file
Create a demo file, `arm_obstacle_demo.py`, with the same setup as our other demos.

# Set up the planning scene
We will use `moveit_python` for its `PlanningSceneInterface` class.
Import `PlanningSceneInterface`:
```py
from moveit_python import PlanningSceneInterface
```

In your main function, instantiate the planning scene and add a virtual "table" obstacle:
```py
# Create table obstacle
planning_scene.removeCollisionObject('table')
table_size_x = 0.5
table_size_y = 1
table_size_z = 0.03
table_x = 0.8
table_y = 0
table_z = 0.6
planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                      table_x, table_y, table_z)
```

In a real setting, you would drive the robot to a tabletop and detect its size and position using perception.
Then, you would add it to the planning scene as a box, as shown above.

Now add a second obstacle:
```py
# Create divider obstacle
planning_scene.removeCollisionObject('divider')
size_x = 0.3 
size_y = 0.01
size_z = 0.4 
x = table_x - (table_size_x / 2) + (size_x / 2)
y = 0 
z = table_z + (table_size_z / 2) + (size_z / 2)
planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)
```

# Visualize the planning scene
For right now, let's just see a visualization of the planning scene we have created so far.
```
rosrun rviz rviz
```

In RViz, add a display and choose "PlanningScene".
You should see your fake table and divider appear:
![image](https://cloud.githubusercontent.com/assets/1175286/25373439/7f507f96-294e-11e7-93c7-7badc060b6c1.png)

For `addBox`, the edge lengths and positions of the box are given in meters.
The position of the box refers to the center.

# Avoid obstacles!
In this demo, we want the robot to move its gripper from one side of the divider to the other:
```py
pose1 = PoseStamped()
pose1.header.frame_id = 'base_link'
pose1.pose.position.x = 0.5
pose1.pose.position.y = -0.3
pose1.pose.position.z = 0.75
pose1.pose.orientation.w = 1

pose2 = PoseStamped()
pose2.header.frame_id = 'base_link'
pose2.pose.position.x = 0.5
pose2.pose.position.y = 0.3
pose2.pose.position.z = 0.75
pose2.pose.orientation.w = 1
```

You do not need to write any special code to avoid obstacles, it is all handled for you by MoveIt!
```py
arm = fetch_api.Arm()
def shutdown():
    arm.cancel_all_goals()
rospy.on_shutdown(shutdown)

kwargs = {
    'allowed_planning_time': 15,
    'execution_timeout': 10,
    'num_planning_attempts': 5,
    'replan': False
}
error = arm.move_to_pose(pose1, **kwargs)
if error is not None:
    rospy.logerr('Pose 1 failed: {}'.format(error))
else:
    rospy.loginfo('Pose 1 succeeded')
rospy.sleep(1)
error = arm.move_to_pose(pose2, **kwargs)
if error is not None:
    rospy.logerr('Pose 2 failed: {}'.format(error))
else:
    rospy.loginfo('Pose 2 succeeded')

planning_scene.removeCollisionObject('table')
planning_scene.removeCollisionObject('divider')
```

# Visualize the plans
Keep RViz open, so that you can see the position of the arm relative to the obstacles.
Add a "Trajectory" display.
Later, when you run your code, you should see a ghostly robot arm previewing the motion plans.
![image](https://cloud.githubusercontent.com/assets/1175286/25377906/ac5cced6-295d-11e7-9660-6c1941eda633.png)

## No obstacles
Try first by commenting out the code that adds the divider to the planning scene.
You should see the robot easily reach the two poses.

## With the divider: attempt 1
Uncomment the divider code and try again.
You should see that with the given parameters, the robot is not able to reach the poses.

## Attempt 2: Set torso height
The robot should fail to find a motion plan relatively quickly.
If you look at the output of `move_group.launch`, you might see messages like:
```
/move_group :298: Found a contact between 'divider' (type 'Object') and 'gripper_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
/move_group :325: Collision checking is considered complete (collision was found and 0 contacts are stored)
```

This means that MoveIt can't find a collision-free path.
Try raising the torso to its maximum height to help the arm get over the divider.

## Attempt 3: Play with parameters
Play around with the parameters and see if you can get the robot to consistently move its gripper from one side of the divider to the other.
Be warned: even with lots of tweaking, it may not always work.
As you play with the parameters, you may find that `move_group.launch` crashes.
In that case, the choice of the parameter values may be overloading the `move_group` node.
Restart it.

## Attempt 4: Change the obstacles
Finally, try changing the position and sizes of the obstacles and see what happens.
Often, you will find that you cannot rely entirely on motion planning to solve all your arm movement needs.
You may need to reposition the robot or program waypoints into your system.

# Attach an object to the robot
When the robot is holding something in its gripper, it needs to ensure that the object does not collide with anything in the environment, either.

MoveIt distinguishes between "collision" and "attached" objects.
Collision objects are parts of the scene to avoid, while attached objects become part of the robot.

We can attach an object to the robot with `moveit_python`:
```py
# Before moving to the first pose
planning_scene.removeAttachedObject('tray')


# If the robot reaches the first pose successfully, then "attach" an object there
# Of course, you would have to close the gripper first and ensure that you grasped the object properly
if ...:
else:
    rospy.loginfo('Pose 1 succeeded')
    frame_attached_to = 'gripper_link'
    frames_okay_to_collide_with = [
        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
    ]
    planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
                             frame_attached_to, frames_okay_to_collide_with)
    planning_scene.setColor('tray', 1, 0, 1)
    planning_scene.sendColors()

# At the end of your program
planning_scene.removeAttachedObject('tray')
```

Note that we specify that the tray (or ruler or whatever the object looks like to you) is attached to the `gripper_link` frame, and is allowed to contact the two fingertip frames.
Otherwise, MoveIt would treat those as collisions.

If you run your program now, you should see the "tray" appear in the robot's hand after it reaches pose 1 successfully.
It then makes a plan to avoid collisions between the tray and the obstacles, as well as with itself.

![image](https://cloud.githubusercontent.com/assets/1175286/25379481/07923f8a-2962-11e7-8eab-8995f15d6536.png)