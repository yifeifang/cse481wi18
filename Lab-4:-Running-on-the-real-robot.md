Because the entire class is sharing one robot, each team will work with the Fetch simulator as much as possible.
However, some things need to be tested in physical reality.
In this lab, we will test whether you are correct passing in the `max_effort` argument to the gripper controller.

# Safety
Before using the robot, read the [Safety](http://docs.fetchrobotics.com/safety.html) guide on the Fetch docs and watch the safety video.
Other tips:
* If you are publishing raw velocity commands, be sure to publish to `cmd_vel` as recommended by the Fetch docs. That way, you can stop the robot at any time by pressing the deadman button (L1) on the joystick.
* Always keep the robot's emergency stop button facing outward where it can be easily reached.
* Always supervise the robot when it is operating. Someone should be able to reach the emergency stop button at all times.
* Press the runstop when you are not using the robot. This prevents you or someone else in the class from accidentally running code that moves the real robot.

# Robot etiquette
There are six teams and one robot, so you must be courteous and try to use the simulator as much as possible.
You must acquire confidence in your code before running on the robot, and you must have a short, focused plan for what you want to test on the real robot.
As soon as you are done with the robot, you must allow others to use it as soon as possible.

To claim the robot, take the joystick and bring it to your desk.

Ways to test your code without the real robot:
* Save several camera images from the real robot and hallucinate them in simulation
* Spawn relevant objects in Gazebo
* Do not use the robot for more than 90 minutes at a time
* Write [unit tests](http://wiki.ros.org/UnitTesting)
* Manually inspect values of messages being published or goals being sent using `rostopic echo`

# Running code on the real robot
ROS nodes will search for a ROS master using the environment variable `ROS_MASTER_URI`.
When you run `roscore` and the Fetch simulator, you are creating a ROS master at localhost:11311.
We have configured the course computer's .bashrc files to show the hostname of the ROS_MASTER_URI in the terminal prompt:
```
[robonaut ~/catkin_ws/src/cse481c (localhost)]$ 
```

To have your ROS nodes search for a different ROS master, use the `setrobot` command:
```
[robonaut ~/catkin_ws/src/cse481c (localhost)]$ setrobot astro
[robonaut ~/catkin_ws/src/cse481c (astro)]$
```

`setrobot` is a convenience command we added to your .bashrc files to set the `ROS_MASTER_URI`.
If you run a ROS node or use a tool like `rostopic list`, it will now contact the robot's ROS master at astro:11311.
Any messages you publish or services you call, or actions you run will now communicate with the robot instead of your simulator.

**Note:** You must run `setrobot` in each terminal window that you want to communicate with the robot.

Once you are done using the robot, you can point your `ROS_MASTER_URI` back to your simulator with:
```
setrobot sim
```

# Test your gripper code
Our plan is to test the gripper by opening the gripper, then closing it with maximum effort around the foam Willow Garage turtle.
Then we will open the gripper and close it with 40N of force.
![img_20170409_193531](https://cloud.githubusercontent.com/assets/1175286/24843959/310a91f8-1d5c-11e7-81f6-6d63b2c04d96.jpg)

1. Make sure your gripper code works in simulation first.
1. Claim the robot by taking the joystick to your desk.
1. Position the robot's gripper so that it is about to pick up the turtle.
1. `setrobot astro`
1. `rosrun applications gripper_demo.py open`
1. `rosrun applications gripper_demo.py close`
1. Look at how much the gripper compresses the turtle with maximum force.
1. `rosrun applications gripper_demo.py open`
1. `rosrun applications gripper_demo.py close 40`
1. The gripper should be barely compressing the turtle, but it should also be able to lift it up without dropping it.
1. `setrobot sim`
1. Return the joystick to the robot.
1. Press the runstop button to disable robot movement.