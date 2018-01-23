This lab mainly outlines what you will need to do for the next assignment.

Create a package called `map_annotator`:
```
cd ~/catkin_ws/src/cse481c
catkin create pkg map_annotator --catkin-deps roscpp rospy
```

The goal of this assignment is to create a package that will allow you to save poses (including both position and orientation) in the map and assign names to those poses.
You should then be able to send the robot to a previously saved pose by name.
Additionally, you should be able to list, overwrite, and delete poses.
The data should be persisted to the disk so that you can retrieve your poses even after shutting down ROS and restarting everything.

In this lab, you will simply develop a command-line app that is easier to implement.

Here is an example of what your program should output:
```
Welcome to the map annotator!
Commands:
  list: List saved poses.
  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.
  delete <name>: Delete the pose given by <name>.
  goto <name>: Sends the robot to the pose given by <name>.
  help: Show this list of commands
> save Main room table
> list
Poses:
  Main room table

> goto Main room table
> delete asdf
No such pose 'asdf'
> delete Main room table
> list
No poses
```

# Persisting data
You can persist data however you want, such as by using [pickle](https://docs.python.org/2/library/pickle.html).

If you are feeling adventurous, you can try using a database, such as [mongodb_store](http://wiki.ros.org/mongodb_store).
mongodb_store converts Python messages into JSON and stores the messages in MongoDB.
If you do go this route, you should launch the `message_store_node` like so:

```xml
<param name="mongodb_use_daemon" value="true" />
<param name="mongodb_host" value="localhost" />
<param name="mongodb_port" value="27017" />
<node name="message_store" pkg="mongodb_store" type="message_store_node.py" output="screen" /> 
```

# Getting the robot pose
In past labs, we used odometry to get the robot pose, but learned that it was inaccurate.
Now, we know that we can use AMCL, which estimates the robot's position based on laser scan data.
To get the robot's location, you should now use the `/amcl_pose` topic.
This topic gives you the robot's position relative to the `map` frame.

The difference between `/map`, `/odom`, and `/base_link` is this:
- `/base_link` is attached to the robot. It represents the center of the robot, on the ground plane.
- `/odom` is where the robot was when it started tracking odometry (usually on startup).
- `/map` is where the robot was when it started making the map.

When you are using AMCL, you can think of the `/map` frame as an arbitrarily chosen center of the world, from which all world coordinates are derived.
Of course, if you change your map, then coordinates you saved for one map will make no sense in the other map.

# Sending the robot to a pose
We know that it is possible to trigger autonomous navigation using RViz.
So, we can repeat the procedure we did in Lab 16 to figure out how RViz set the initial pose.

Here are some questions to guide you:
- Does RViz use a topic, a service or an action to start autonomous navigation?
- What node is RViz communicating with? Is it AMCL or a different node?
- If you are unsure, what documentation can you look up to help you find the answer?

# Final notes
Keep in mind that in the next lab, you will need to do essentially the same thing, but with a graphical interface.
Think of the command-line interface as a kind of "frontend" and try to separate your frontend code from your backend code so that you can reuse your backend code for the next lab.