In this lab, we will learn to use the pre-built navigation capabilities included with the Fetch robot.

Researchers and engineers have worked on the robot navigation problem for a long time, leading to the development of the [ROS navigation stack](http://wiki.ros.org/navigation).
The Fetch robot uses this stack as well.

# Overview
Navigation can be broken down into several subproblems, including localization, mapping, and path planning.

Localization is the process of determining where the robot is in the world.
Odometry is one way of tracking the robot's position, but, as you might have noticed in the previous labs, it is not very accurate.
A better approach is to fuse information from additional sensors.
The Fetch robot, along with many other robots, has a laser scanner in its base to help with localization.

A common algorithm for localization is called Monte Carlo Localization, also known as a particle filter.
In this algorithm, particles represent a guess at the robot's position.
For each laser scan, the algorithm estimates the probability of each particle seeing that data.
Particles that seem unlikely to have produced that data die off, while more plausible particles are boosted.
[amcl](http://wiki.ros.org/amcl), which stands for Adaptive Monte Carlo Localization, is the most commonly used open-source implementation of this algorithm within the ROS community.

In order to localize the robot, it needs to have a map of the environment.
While floor plans can be used as the map, they can not always be obtained.
Instead, the robot itself can map the environment using its laser scanner.
This is done through a process known as SLAM, or simultaneous localization and mapping.
Some common packages for SLAM in ROS are [karto](http://wiki.ros.org/slam_karto) (used by the Fetch), [gmapping](http://wiki.ros.org/gmapping), and a relatively new system called [cartographer](http://wiki.ros.org/cartographer).
In ROS, maps are often represented using images, which can be edited in image editors to clean up noise.

Finally, the robot needs to be able to plan paths to its destination and avoid obstacles while moving.
This can be accomplished using variants on A* planning.
The Fetch robot also supports "keep out" zones, which specify to the planner that the robot should not go there.
However, the "keep out" map must be separate from the map that is used for localization.
The common ROS navigation stack package for doing planning and obstacle avoidance is [move_base](http://wiki.ros.org/move_base).

For Fetch navigation in particular, you can refer to the [Fetch docs navigation tutorial](http://docs.fetchrobotics.com/navigation.html).

# Building a map
Make sure that the Fetch simulator is running.
Then run:
```
roslaunch fetch_navigation build_map.launch
```

This starts the Karto SLAM node.

## Configure RViz
We will want to have different RViz configs for different purposes.
What to visualize while navigation will be different from what to visualize when working on perception or manipulation.
We can even have multiple instances of RViz open, each with its own configuration.

For building a map, we will want to visualize the following things:
- The robot model
- A grid
- The map
- The laser scan
- An image from the head camera
- (Optional) The point cloud from the depth sensor
- The fixed frame should be set to `map`

Open RViz and configure it to have those displays.
Your RViz window should look like this when done:
![image](https://cloud.githubusercontent.com/assets/1175286/25209079/7d620cf6-252d-11e7-9dfd-1264bd9eb82a.png)

You will also want to commit your RViz config files to your code repository, so that everyone else who uses your code can see the same RViz config.
Typically, we save the RViz config file to a folder named `config`.
Save your config file to `~/catkin_ws/src/cse481c/applications/config/navigation.rviz`.

## Configure a launch file
In this section, we will see how to create a launch file to automate the process of launching multiple things.
In this case, the build a map, we need to run or launch three separate things:
- `roslaunch fetch_navigation build_map.launch`
- `rosrun rviz rviz -d ~/catkin_ws/src/cse481c/applications/config/navigation.rviz`
- `rosrun applications keyboard_teleop.py`

Let's create a launch file in `applications/launch/build_map.launch` that does all three of these at once:
```xml
<launch>
  <include file="$(find fetch_navigation)/launch/build_map.launch" />
  <node pkg="rviz" type="rviz" name="$(anon rviz)" args="-d $(find applications)/config/navigation.rviz" />
  <node pkg="applications" type="keyboard_teleop.py" name="fetch_keyboard_teleop" output="screen" />
</launch>
```

You can read about how to set up launch files in the [roslaunch documentation](http://wiki.ros.org/roslaunch/XML).
The example above illustrates the use of several features.

The first line shows how to launch a different launch file.
Launch files support substitution arguments, such as `$(find PKG_NAME)`, which will be replaced with the directory of a package.

The second line shows how to run RViz with your navigation.rviz config file.
You have to pass in the path to the config file using the -d option.
Note that we use `$(find applications)` instead of hardcoding the location `/home/teamN/catkin_ws/src/cse481c/applications`.
Also note that we give RViz an "anonymous" name using `$(anon rviz)`.
In ROS, if two nodes have the same name, the older node will be shut down.
Anonymous names are guaranteed to be unique, so you can launch this launch file without worrying that it will shut down a different instance of RViz that you might have running.

Finally, the third line shows how to run the keyboard teleop app.
We set `output="screen"` to guarantee that its output will reach standard output, which is generally necessary for applications that read from standard input.

## Do the mapping!
Shut down RViz and the Karto node if they are already running.
With your launch file, you can now run:
```
roslaunch applications build_map.launch
```

With the terminal open next to RViz, you can now drive the robot around and watch it build up a map of the world.
Drive around until your map is complete.
Once it is complete, run:
```
mkdir ~/maps
rosrun map_server map_saver -f ~/maps/playground
```

This will save your map to ~/maps/playground.yml and ~/maps.playground.pgm.
The .pgm can be edited in an image editor like Gimp to clean up noise.

![image](https://cloud.githubusercontent.com/assets/1175286/25209747/d457975c-2531-11e7-81bd-7796f383859c.png)

## Mapping with the real robot
Mapping with the real robot is similar to mapping in simulation.
In this lab, you will not do any mapping because the lab space is too crowded.
However, in the future, we might setup a robot workspace that you need to map.

Here are some tips:
- Start with the robot aligned with the walls (i.e., not at a diagonal)
- Use the joystick to teleop instead of the keyboard (you can still leave keyboard teleop running)
- You may want to run `build_map.launch` (sans Rviz) on the robot itself (as opposed to your lab computer) in case it loses its wifi connection as it drives around
- Give the mapping plenty of opportunities to "close the loop," i.e., see something that it has seen before. That will make the mapping more accurate.
- In simulation, the simulated data almost perfectly accurate, so the maps turn out very clean, but in the real world, the maps will look a lot messier. It may take multiple tries to get a good map.
- Edit your .pgm file afterwards to clean up noise and erase ephemeral objects that shouldn't be in the map. For example, if someone was standing still in some location while you were mapping, they might become part of the map.

# Sending navigation goals in RViz
Next, we will see how to send navigation goals to the robot in RViz.

## Set up a launch file
First, shut down `build_map.launch`, as Karto is not supposed to be running while navigation is running.
To do this, we need to launch RViz and the Fetch navigation launch file, `fetch_nav.launch`.

First, let's see what's in `fetch_nav.launch`:
```
rosed fetch_navigation fetch_nav.launch
```

This launch file takes in several arguments, including `map_file`, which specifies which map the robot should use.
Notice that it provides a default value.
That map is not what we want, so we will need to pass in a different value for `map_file`.
Note also that the Fetch navigation launch file will tilt the robot's head up and down, which is used for detecting obstacles.

Create a launch file in `applications/launch/nav_rviz.launch` that launches `fetch_nav.launch` and RViz with the `navigation.rviz` config.

```xml
<launch>
  <include file="???" />
  <node pkg="rviz" type="rviz" name="???" args="???" />
</launch>
```

However, recall that we have to override the `map_file` arg to `fetch_nav.launch`.
Look at the documentation for [roslaunch XML](http://wiki.ros.org/roslaunch/XML) and scroll down to Section 4: Tag Reference.
Next, click on `<include>`, which leads to the detailed documentation of how to use the `<include>` tag.
The documentation states that you can pass in arguments using the `<arg>` tag.
You should also look at the documentation for the `<arg>` tag to see how to use it.

In your `nav_rviz.launch`, add an argument for `map_file` with a default value that points to the map we just created.
Then, pass that arg to the `<include>` tag for `fetch_nav.launch`.
The documentation on [substitution args](http://wiki.ros.org/roslaunch/XML#substitution_args) may also be helpful.

```xml
<launch>
  <arg name="map_file" default="???" />
  <include file="???">
    <arg name="map_file" value="???"
  </include>
  <node pkg="rviz" type="rviz" name="???" args="???" />
</launch>
```

Run the launch file.

## Updating the RViz config
Let's now update our RViz config to show more information specific to navigation.
Add the following displays to RViz:
- The particles from AMCL (a PoseArray)
- The global path plan (a Path, change the color to blue)
- The local plan (a Path, keep the color green)
- The local cost map (a Map, change the color to "costmap" and set "Draw behind" to true)
- The global cost map (a Map, set "Draw behind" to true and lower the alpha to 0.3)

Notice that you can give names to the displays that you add by clicking on a display and clicking "Rename."
Save your config file.

You should now see something like this:
![image](https://cloud.githubusercontent.com/assets/1175286/25210532/3124238a-2535-11e7-979b-6bf61de13f4b.png)

## Global localization
At this point, the robot is probably in the wrong location on the map.
When you first start up, the robot doesn't know where it is to start with.
There are two approaches to solving this:
- Global localization, in which AMCL attempts to localize itself automatically by spreading particles uniformly throughout the map
- Manual input, in which the user inputs the location of the robot manually

We will try the first approach.

Type `rosnode info /amcl` and notice what services it implements.
You should see a service for global localization.
Use `rosservice info` to see the service's type.

Call the service using `rosservice call` and see what happens to the particles in RViz:
![image](https://cloud.githubusercontent.com/assets/1175286/25210648/c80bff5c-2535-11e7-8c51-59e78f73580c.png)

Now try driving the robot around a bit with keyboard teleop and see if the particles converge to the correct location or not.
You can try calling the global localization service from different starting points.

Unfortunately, you probably will notice that global localization does not work unless you call the global localization service while the robot is inside one of the two rooms.
The distinct shape of the rooms helps the robot quickly realize that it's in one of the rooms, and the particles eventually converge to the correct location.
However, if you are in the hallway or out in the open, then the robot just sees a bunch of walls at different distances.
That's not enough information for the robot to realize where it is.
And, once the particles converge on the wrong location, they tend to stay wrong.

## Manual localization
Let's now try giving the robot an initial pose estimate manually.
- Call the global localization service again. This should scramble the robot's location in RViz.
- Now, look at where the robot actually is in Gazebo.
- In RViz, click on the "2D Pose Estimate" button in the top toolbar. Click once on the map to set the robot's location, and click again to set its orientation.
- You should see the laser scan align roughly with walls of the map.

![image](https://cloud.githubusercontent.com/assets/1175286/25211104/47a7841e-2538-11e7-950e-4afbc718d5b9.png)

Now, if you drive the robot around, it should easily stay localized.
The downside of this approach is that it involves human intervention and that it requires you to know where the robot actually is, which may not be the case if you are operating the robot remotely.

## Hunting: How does RViz do it?
If you know that the robot is at a particular location, such as its charging station, you can also set an initial pose programmatically.
The question is how.

In this section, we will practice hunting.
This is a good skill to have and the next section will require you to do something similar.

We know that we can set the initial pose via RViz, and RViz is somehow communicating with AMCL.
The three modes of communication in ROS are topics, services, and actions.
If RViz is using topics or actions, then we should see it publishing to a topic that AMCL subscribes to.
If RViz is calling a service then we should see AMCL offering a service that sets the initial pose.

Run `rosnode info /rviz-...`, where you will need to use tab completion to get the name of the RViz node.
In another terminal, run `rosnode info /amcl`.
We can see that RViz publishes to a topic called `/initialpose`, which sounds promising.
Our suspicions will be confirmed by running `rostopic info /initialpose`, which shows that AMCL is subscribing to it.
So, to set the initial pose programmatically, we just publish a message on this topic.

To get a final confirmation, just `rostopic echo /initialpose` and then set a 2D pose estimate in RViz again.
You should see a message being published, which confirms our hypothesis, and also shows us a sample message.

## Play with navigation
We saved the fun part for last.
Now that the robot has a map and is localized, it can autonomously navigate around the room.
To do this, go to RViz and click on the "2D Nav Goal" button.
Click someplace on the map and you should see the robot navigate there.

A few things to notice:
- If you set up RViz correctly, you should see a blue trail and a green trail. The blue trail is the "global" plan while the green trail is the "local" plan.
- The costmap is like an inflated version of the map. Notice that the robot drives very slowly when in high cost areas, such as through a doorway.

The global plan and costmap relate to how the robot sees the world just from the map data.
The local plan and costmap incorporate dynamic factors, such as obstacles that weren't in the map (e.g., people walking around).

![image](https://cloud.githubusercontent.com/assets/1175286/25211613/d40bfb0e-253a-11e7-9772-da9fbf735051.png)