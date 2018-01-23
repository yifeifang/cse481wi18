So far, we have treated RViz purely as an output node that visualizes information published by other nodes.
Next, we will see how RViz can be used to get input from the user.
This is done with the use of [InteractiveMarkers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started).
InteractiveMarkers augment regular Markers by allowing the user to change the position or rotation of the Marker, handle click events on the Marker, or allow them to select something from a context menu assigned to the Marker.
To create an InteractiveMarker, you will need to write an InteractiveMarker server that publishes the InteractiveMarker and handles the input given through RViz.

# Interactive markers tutorials and documentation
You can find tutorials on interactive markers in the main [rviz Tutorials](http://wiki.ros.org/rviz/Tutorials).
To better understand how InteractiveMarkers work, read tutorial 3: [Interactive Markers: Getting Started](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started).

The two tutorials after that show how to create an interactive marker server in C++.
However, we will use Python in this lab.
Much of what you see in the C++ tutorials will be identical in Python.
However, you need to find the API documentation for Python.
To do this, follow these steps:
- Find documentation on interactive markers by doing a web search for "ROS interactive markers".
- You should be taken to the `interactive_markers` package documentation.
- Click on "Indigo" and then click on the "Code API" link in the grey box. Most ROS packages have similar links to their Code API in the same location.

  ![image](https://cloud.githubusercontent.com/assets/1175286/25160451/fad2bd16-246a-11e7-919d-94b79558bcea.png)
- You will find that the `interactive_markers` documentation weirdly combines C++ and Python in a single website. To find Python documentation, click on "Namespaces", then click "interactive_markers::interactive_marker_server", then finally click "InteractiveMarkerServer". This is the documentation for the Python class `interactive_markers.interactive_marker_server.InteractiveMarkerServer`. You can navigate the site further to find other documentation.

It is also useful to look at the definition of the `InteractiveMarker` message.
To do this, follow these steps:
- Do a web search for `InteractiveMarker`
- Click on the first result and change `kinetic` in the URL to `indigo`

Alternatively, use `rosmsg show visualization_msgs/InteractiveMarker -r`

# Creating an InteractiveMarker Server

Create a new script called `interactive_marker_demo.py` in the `applications` package.
Let's start with something basic.
First, be sure to import the following.

```py
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
# ... Other imports ...
```

First, create the server.
```py
server = InteractiveMarkerServer("simple_marker")
```

Then, create an `InteractiveMarker`.
```py
int_marker = InteractiveMarker()
int_marker.header.frame_id = "base_link"
int_marker.name = "my_marker"
int_marker.description = "Simple Click Control"
int_marker.pose.position.x = 1
int_marker.pose.orientation.w = 1
```

Next, create a teal cube `Marker` for the `InteractiveMarker`.
```py
box_marker = Marker()
box_marker.type = Marker.CUBE
box_marker.pose.orientation.w = 1
box_marker.scale.x = 0.45
box_marker.scale.y = 0.45
box_marker.scale.z = 0.45
box_marker.color.r = 0.0
box_marker.color.g = 0.5
box_marker.color.b = 0.5
box_marker.color.a = 1.0
```

Next create an `InteractiveMarkerControl`, add the `Marker` to it, and add the control to the `InteractiveMarker`.

```py
button_control = InteractiveMarkerControl()
button_control.interaction_mode = InteractiveMarkerControl.BUTTON
button_control.always_visible = True
button_control.markers.append(box_marker)
int_marker.controls.append(button_control)
```

Next, write the callback function that will handle the input received through RViz.

```py
def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')
```


The final lines of code you need to add in your main function will InteractiveMarker to the server with the callback information.
```py
server.insert(int_marker, handle_viz_input)
server.applyChanges()
```

To test the interaction with your InteractiveMarker, run your script alongside RViz.
In RViz, add an InteractiveMarker display and choose the topic (`/simple_marker/update`).
When you click on the box, you should see output in the console.

![image](https://cloud.githubusercontent.com/assets/1175286/25161306/0acf71fa-2470-11e7-9202-51a51088b664.png)

# Other things you can do with Interactive Markers
This lab demonstrates the use of a very simple interactive marker.
However, there are many more things you can do.

**Things to try:**
You do not need to implement these, but look at the available documentation and think about these questions:
- Look at Robot Web Tools's [online demo of interactive markers](http://robotwebtools.org/demos/im.html)
- Look at the Code API. How do you erase a marker from the server?
- How do you add a right-click menu to a marker?
- Our demo responds to `InteractiveMarkerFeedback.BUTTON_CLICK` events. What other events are there?

# Triggering Continued Robot Actions though InteractiveMarkers

Different controls for InteractiveMarkers allow obtaining different types of input through RViz.
Before going into other types of controls we would like to further explore how to handle simple click inputs and how handling of events can impact the architecture of your code.

To that end, you will extend your script to publish three different InteractiveMarkers at different locations in the room. Clicking on an interactive marker should trigger the robot to move towards that InteractiveMarker.
You can make the robot move towards a known location by first rotating towards the target and then moving on a straight line towards it until you are close enough.
If another marker is clicked while the robot is moving, it should change course and start moving towards the newly clicked InteractiveMarker.

Based on this lab, you should be able to implement the interactive marker display and handling of clicks.
To accomplish the driving portion, please see the next two labs, [[Lab 14: Odometry and rotations]] and [[Lab 15: Driving with odometry]].