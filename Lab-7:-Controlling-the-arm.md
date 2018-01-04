In this lab, we will look at a simple way to control the arm.
You may not end up using this method when working on your projects, but this will show you what you can achieve at a low level.

# Joint space vs. Cartesian space
There are two common ways to control the position of the arm: joint space and Cartesian space.
In joint space, we control the 7 degrees of freedom of the arm directly.
In Cartesian space, we specify where we want the end-effector (the gripper) to be and how it should be oriented.

In the future, we will show you how to control the end-effector in Cartesian space, as it will probably be more useful to you.
However, even Cartesian space commands will eventually be transformed into joint space.
And, because there are multiple ways the joints can be configured to achieve the same end-effector pose, sometimes the exact joint space configuration is important to specify.
In this lab, you will send joint space commands to the arm.

# Read the arm documentation
Read the Fetch docs on the [Arm API](http://docs.fetchrobotics.com/api_overview.html#arm-and-torso).
Notice that it doesn't have much to say.
This is because controlling the arm is basically identical to controlling the torso or the head: you specify a trajectory for the 7 joints of the arm to follow.

So, if you have completed the torso controller, the code for controlling the arm will be virtually the same.
The main difference is that instead of controlling one joint (the torso), you will be controlling seven joints at the same time.

# `ArmJoints`
For your convenience, we have created a class called `ArmJoints`, which is like a struct that holds joint values for the arm.
You can initialize `ArmJoints` from a list, or set joint values one by one:
```py
joints1 = ArmJoints()
joints1.set_shoulder_pan(1.5)
...

joints2 = ArmJoints.from_list([1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0])
```

Once you have an `ArmJoints` object, you can access the names and values in order:
```
ArmJoints.names()  # Returns the names of the arm joints.
joints.values()  # Returns the joint values stored in the object, in the same order as ArmJoints.names()
```

# Implement arm movement
We have implemented a demo file for you in `applications/scripts/arm_demo.py`.
You only need to implement the missing code in `fetch_api/src/fetch_api/arm.py`.
Make your trajectories take 5 seconds each.

# Test in simulation
**You will need to raise the torso of the robot to its maximum height before running the arm demo.
Otherwise, there will be collisions with itself and the ground.**

The arm demo will move the arm across almost its entire range of motion.
You should test your demo in simulation to see for yourself.
Do not test your arm code on the real robot, as it is too crowded in the lab to run the demo safely.