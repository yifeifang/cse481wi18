# actionlib
To control the gripper (and most other interfaces of the robot), you will need to be familiar with [actionlib](http://wiki.ros.org/actionlib) and the [actionlib tutorials](http://wiki.ros.org/actionlib/Tutorials).

Do the actionlib beginner tutorials if you have not already.

# Gripper interface
Go to the [Fetch docs](http://docs.fetchrobotics.com/index.html) and click on *API Overview* -> *Gripper Interface*.
The robot runs an actionlib server to control the gripper, so you will need to write an actionlib client to send open/close commands.

If you look at the [GripperCommand](http://docs.ros.org/api/control_msgs/html/action/GripperCommand.html) action linked from the Fetch docs, you will see that the goal msg has just one field, `command`, which is confusingly of type `GripperCommand`.
No, this is not a circular definition.
`GripperCommand` is also the name of a *message* as well as an *action*.
You can see what a GripperCommand message looks like by clicking on the `control_msgs` link in the GripperCommand action page:
![image](https://cloud.githubusercontent.com/assets/1175286/24843019/7e718f40-1d53-11e7-9f9b-e48e1caefaa2.png)

From there, you can see a list of all the messages, services, and actions defined in the `control_msgs` package:
![image](https://cloud.githubusercontent.com/assets/1175286/24843039/9d00c5ca-1d53-11e7-965f-8af6df0e53ae.png)

Note that the name GripperCommand is used twice, once as an action and once as a message.

So to create a goal, your code will look something like this:
```py
goal = control_msgs.msg.GripperCommandGoal()
goal.command.position = CLOSED_POS
goal.command.max_effort = max_effort
```

# Write the action client
In the course repo, a skeleton of the gripper code has been written for you in `fetch_api/src/fetch_api/gripper.py`.
Fill in the methods `open` and `close`.

# Finish the demo
The course repo contains a partially implemented file you can use to test your code in `applications/scripts/gripper_demo.py`.
Fill out the sections that simply print "Not implemented."

If you run the demo with:
```
rosrun applications gripper_demo.py
```

You should see the error:
```
[rosrun] Couldn't find executable named gripper_demo.py below /home/teamN/catkin_ws/src/cse481c/applications
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   /home/teamN/catkin_ws/src/cse481c/applications/scripts/gripper_demo.py
```

Any ROS Python file with a main function should be marked executable:
```
chmod +x gripper_demo.py
```

Now you should be able to run the demo, and see the robot's gripper open and close in the simulator:
```
rosrun applications gripper_demo.py close
rosrun applications gripper_demo.py open
```