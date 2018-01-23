Visualizations of different sensor data and hidden robot states can be extremely helpful in debugging robot software.
[RViz](http://wiki.ros.org/rviz) is a visualization tool in ROS that allows creating visualizations from published messages of certain types.

# Getting started with RViz

Check out the [RViz User Guide](http://wiki.ros.org/rviz/UserGuide) to see everything you can do with RViz.
Let's try a few of its functionalities.

First, fire up the Fetch Gazebo simulator in the "playground" setting. Then open a new terminal and run RViz:

```
rosrun rviz rviz
```

An empty RViz window should appear.
At this point you should see a "Displays" subwindow on the left that includes only the "Global Options," "Global Status," and a "Grid" display.
The dark gray subwindow in the middle is the 3D display.
Different visualizations can be aggregated on the 3D display, as long as they have a common or linked frame of reference. You can ignore the rest of the RViz window for now.
In this lab we will visualize things that are anchored to the `base_link` or `odom` so you should switch the "Fixed Frame" in global options to either of those.

![image](https://cloud.githubusercontent.com/assets/1175286/25155859/309204e4-244c-11e7-97c1-9823cb046567.png)

# Adding Elements to RViz

Next, you will add different visualization elements, called "Displays."
Click on the "Add" button and explore the list of different display types offered in the pop-up.

**Note:** If the display panel does not update or react to clicks, you may need to resize the RViz window to have it re-render the UI.
This is a known issue.

Start by adding a RobotModel.
A new item should appear in the "Displays" list and a visualization of the Fetch robot should appear in the 3D display. 
Expand the display options to explore the different parameters of the Grid and RobotModel displays.
While the grid visualization does not depend on any data, the RobotModel display is configured by reading the  `robot_description` param.

Please note that RViz is not a simulation of the robot, but rather it is a visualization of the robot that is currently simulated in Gazebo.
You can use the robot teleoperation tool you developed last week to change the state of the robot in Gazebo and observe that the change is reflected in RViz.
If you use `base_link` as the Fixed Frame you might not notice the base movements, whereas if you use `odom` the robot will get displaced from the center of the grid.

Next, add a LaserScan and a PointCloud2.
In the new displays added to the Displays list on the left, click on the space next to the "Topic" box to reveal the list of potential topics for those displays.
Different RViz displays are designed to visualize different types of topics.
Find the right topics to visualize and configure them so you can differentiate between the two sensor data.

The laser scan will visualize a 2D slice of the world from the robot's base.
Compare what you see in RViz to the world in Gazebo.

The point cloud display will show the 3D data from the robot's depth sensor (i.e., the robot's eyes).
To see in color, use the topic, `head_camera/depth_registered/points` (*registered* means that the depth and color images are merged together).
If you see the point cloud but it is while, you may need to change the "Color Transformer" field to "RGB8."
You can use the robot teleoperation tool you developed last week to move the robot’s head around and observe how the visualized point cloud changes.
You can also interact with the 3D display change the perspective and zoom, to view the sensor data differently.

![image](https://cloud.githubusercontent.com/assets/1175286/25156187/df2d5fa2-244d-11e7-8910-0bc2cae8e43f.png)

In addition to 3D visualizations, you can add visualizations in separate subwindows within RViz. For example, try adding an Image or a Camera, with different topics that are available.

Now that you get the idea of what RViz is you are welcome to further explore other display types before moving on to creating customized visualizations in the 3D display.

# Saving and loading RViz configurations

You do not want to manually add all these displays every time you use RViz.
Instead, you should save your current configuration.
To save your current configuration as the default (in ~/.rviz/default.rviz), just go to `File->Save Config` or press Ctrl+S.
Try saving and reloading RViz and make sure that your configuration comes up as expected.

Later, you will want different RViz configurations for different purposes.
You can save your current configuration to a separate .rviz file and switch between different configurations.
To do so, go to `File->Save Config As`.
Your config file will be saved in `~/.rviz/myconfig.rviz`.
Later, you can load this config file by going to `File->Open Config`.

You also may want to run RViz with a certain config from the command line or from a launch file.
To open RViz with a particular config file, run:
```
rosrun rviz rviz -d ~/.rviz/myconfig.rviz
```

To open RViz from a launch file with a particular config file, add this to your launch file:
```xml
<node pkg="rviz" type="rviz" name="$(anon rviz)" args="-d $(env HOME)/.rviz/myconfig.rviz" />
```

Try saving and loading a custom RViz config now.

# RViz with the real robot

RViz can similarly be used to visualize the current state of the real physical robot and its sensor data.
To try it, open a new window, use the command `setrobot astro` to start communication with the real robot and run RViz in the same terminal.

**DO NOT STREAM A POINT CLOUD FROM THE REAL ROBOT**

This has caused problems in the past.
The robot has limited wireless network bandwidth and generally cannot stream point clouds across the network.
When you work on perception in later labs, your code will run directly on the robot, where there are no bandwidth/latency issues.
Instead, you can stream an Image display, or you can look at the **downsampled** point cloud. 
