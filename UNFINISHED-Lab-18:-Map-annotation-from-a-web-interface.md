This lab will also guide you through parts of your next assignment.

The goal of this lab is similar to the previous lab: we want an interface for creating poses on a map and sending the robot to them.
However, we want a more user friendly interface than just a command-line interface.
Additionally, it would be nice to annotate poses on the map without having to drive the robot there.

# Overview
Our system will have a frontend and a backend component.
This time, however, the frontend will be a web app.
We will use [`<ros-rviz>`](https://www.webcomponents.org/element/jstnhuang/ros-rviz), a highly experimental, web-based version of RViz developed by the University of Washington [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/).

Because `<ros-rviz>` is experimental, you can feel free to use "real" RViz instead.
Using the web-based version has the advantage of working on all operating systems and on mobile devices.
However, it may be buggy, and it doesn't support high-bandwidth visualizations like point clouds.

The app will split responsibilities between buttons displayed on the web page and interactive markers that will appear in RViz:

| **Feature** | **Buttons** | **Interactive markers** |
| --- | --- | --- |
| **Create pose** | User clicks "Create Pose" button. | User places interactive marker. |
| **List poses** | List of poses displayed in interface. | |
| **Delete pose** | User clicks "Delete" next to pose. | |
| **Send robot to pose** | User clicks "Go here" next to pose. | |
| **Edit pose** (optional) | | User drags interactive marker. |
| **Rename pose** (optional) | User renames in web interface. | |

# Latched topics
"Reactive databases" like [Firebase](https://firebase.google.com/docs/database/) or [Meteor](https://www.meteor.com/) are a hot trend in web development today.
In these systems, changes made to the data are immediately propagated to all other clients viewing the same data and updated in real time, without having to refresh the page or periodically poll the server.
Under the hood, these interfaces are implemented using publish/subscribe systems.
Fortunately, we can implement some of the features of reactive databases using ROS.

One way to simulate a reactive database is to publish/subscribe to a "latched" topic.
You can read in the [rospy Publisher](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers) documentation that with a "latched" topic, the last message sent will be saved and sent to any nodes that subscribe to that topic, even if they started subscribing after the message was published.
In this way, subscribing to this topic is like reading a particular value from a database and subscribing to future updates to that value.
To create a latched topic, add `latch=True` when you create the `Publisher`.

We will publish the list of poses to a latched topic.
You will need to create a msg called `PoseNames`, which is just a list of strings.
Then, your server needs to publish the list of pose names whenever a pose is added, deleted, or renamed.
Finally, your website should subscribe to this topic, and re-render the list of poses whenever it changes.

Note that whenever you create a new message type, you will need to shut down `rosbridge_websocket.launch` and re-source your environment before restarting it:
```
# ... Create new message ...
catkin build
# ... Shut down rosbridge_websocket.launch ...
source ~/.bashrc
roslaunch rosbridge_server rosbridge_websocket.launch
```

# UserAction
One technique for a web interface to communicate with the server is to treat the user's interaction with the interface as a stream of actions.
In order words, you can publish a message to a topic (e.g., `/user_actions`) whenever the user takes some action in the interface (clicks the "Create" button, clicks a "Delete" button, etc.)
In this way, you can test your backend by just publishing messages to the `/user_actions` topic, even if the frontend isn't finished yet.
This topic can also be recorded, analyzed, and played back for testing purposes.

In this interface, all of the user actions can be specified with a `command` parameter that acts on a pose `name`.
If you want to support renaming a pose from the web interface, you may also want to add an additional parameter, `updated_name`.
Your `UserAction.msg` can look like this:

```
string CREATE=create
string DELETE=delete
string GOTO=goto
# string RENAME=rename
string command
string name # The name of the pose the command applies to
string updated_name # If command is RENAME, this is the new name of the pose
```

At this point, your team should theoretically be able to work in two groups: one group that focuses on developing the web interface and another that focuses on developing the backend.
To finish the web interface, you will need to instrument the interface such that it publishes the correct UserAction messages in response to button clicks.
To work on the backend, you will need to subscribe the the `/user_actions` topic and add or remove poses as requested by the UserAction.
A third component your team members can work on is the interactive marker interface.

Below, we give a few instructions for the backend team, but the majority of the instructions are for the web frontend.

# Download prereqs
```
sudo apt-get install ros-indigo-rosbridge-server ros-indigo-tf2-web-republisher ros-indigo-interactive-marker-proxy
```

# Launch file
Create a launch file for your team:

```
cd ~/catkin_ws/src/cse481c/map_annotator
mkdir launch
vim launch/map_annotator.launch
```

**map_annotator.launch**
```xml
<launch>
  <!-- Map annotator server -->
  <!-- You can pass command-line args to your server using the args attribute below. -->
  <node pkg="map_annotator" type="server.py" name="map_annotator_server" args=""/>

  <!-- Web prereqs -->
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" />
  <node pkg="tf2_web_republisher" type="tf2_web_republisher" name="tf2_web_republisher" />
  <node pkg="interactive_marker_proxy" type="proxy" name="interactive_marker_proxy_basic_controls">
    <remap from="target_frame" to="/map" />
    <remap from="topic_ns" to="/map_annotator/map_poses" />
    <param name="update_rate" value="10.0" />
  </node>
</launch>
```

# Interactive marker / backend team
## Interactive marker interface
Once a user creates a marker and gives it a name in the web interface, a new interactive marker should appear in (web) RViz at the ground level at the (0, 0, 0) position and unit orientation.
The marker should be an arrow, so that the user can tell which way the Fetch will be facing in this pose.
The user should be able to change the marker's position and orientation.
As the pose of the marker is changed, the database (or whatever data structure you are using to store the poses) should be updated.
And, if a pose is deleted from the web interface, the marker for that pose should disappear from RViz as well.

**Hint:**
Look at the *Chess Piece* marker in the [Interactive Markers Basic Controls tutorial](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls) to get an idea of how to create a marker that can be dragged in the XY plane.
To get it to rotate, you will need to add another control to the interactive marker, see the [InteractiveMarkerControl definition](http://docs.ros.org/indigo/api/visualization_msgs/html/msg/InteractiveMarkerControl.html) and figure out what that is.
This control will not need any markers, because a rotation control renders a draggable ring by default.

**Hint 2**:
Your markers will be easier to click if you make the controls slightly off the ground.
That way, the map visualization doesn't interfere with it.

Here is an example of what your interactive marker might look like:
![image](https://cloud.githubusercontent.com/assets/1175286/25220378/6c188fe0-2566-11e7-9735-c84cd1f6ee11.png)

## Persisting data to disk
The easiest way to do this is by simply using [pickle](https://docs.python.org/2/library/pickle.html) to save your data structure of poses to disk.
You can register a function to run when the node is shutdown as described in [rospy: Initialization and Shutdown](http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown).
Remember to reinitialize everything (web interface pose list, interactive markers) when the server starts up.

# Frontend team
Create the frontend:
```
cd ~/catkin_ws/src/cse481c/map_annotator
mkdir frontend
cd frontend
polymer init
```

In the menu:
- Pick `polymer-2-application` in the menu that appears.
- Name your application `map-annotator`
- Accept the remaining defaults

Now add the ROS web components as dependencies:
```
bower install --save jstnhuang/ros-websocket
bower install --save jstnhuang/ros-topic
bower install --save jstnhuang/ros-rviz
```

`bower_components/` now contains a ton of files, which should be downloaded via bower and not stored in Git.
```
echo "bower_components/" >> .gitignore
```

As a reminder, you can find documentation for these three elements on https://webcomponents.org:
- [`<ros-websocket>` API docs](https://www.webcomponents.org/element/jstnhuang/ros-websocket/elements/ros-websocket)
- [`<ros-topic>` API docs](https://www.webcomponents.org/element/jstnhuang/ros-topic/elements/ros-topic)
- [`<ros-rviz>` API docs](https://www.webcomponents.org/element/jstnhuang/ros-rviz/elements/ros-rviz) and [User Guide](https://github.com/jstnhuang/ros-rviz/wiki/User-guide)

Run your web app and the map_annotator launch file:
```
roslaunch map_annotator map_annotator.launch # Remember to re-launch whenever you add/edit messages
polymer serve -H 0.0.0.0
```

## "Flatten" your application (optional)


## Connect to the websocket server
Connect to the websocket server similar to how you did in Lab 10.

## Show the pose list


Visit localhost:8080 in a web browser and open the JavaScript console.
Now try publishing some latched messages to the `pose_names` topic:
```
rostopic pub /pose_names map_annotator/PoseNames "names:
- 'Test 1'
- 'Test 2'"
```
You should see "Test 1" and "Test 2" appear in the pose list with "Go to" and "Delete" buttons.

![image](https://cloud.githubusercontent.com/assets/1175286/25216800/21d53aee-2559-11e7-8c9c-de15cce503b3.png)

# Testing on your phone
Once your tool is working, you should be able to load the webpage from your phone by visiting `IP_ADDRESS:8081/` in a web browser.
You can find your course computer's IP address by typing `ifconfig` in a terminal.
Deleting a pose on your phone should be reflected on the desktop computer, and vice versa.

# Final result
Here is a video showing how your interface might look when done.
Note that changes are synchronized between different browsers and RViz.
Also note that when the backend is restarted, all the poses reappear in the same locations.
[![image](http://i3.ytimg.com/vi/ZoHjurYzME0/hqdefault.jpg)](https://youtu.be/ZoHjurYzME0)