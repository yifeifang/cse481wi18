In this lab, you will create custom visualizations in the RViz 3D display.
This is done with the help of [Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker), which are a special type of 3D display you might have noticed in the previous lab.
The Markers display allows programmatic addition of various primitive shapes to the RViz 3D view by sending a `visualization_msgs/Marker` or `visualization_msgs/MarkerArray` message.

# Add a marker display to RViz
Add a "Marker" display to RViz.
Note that it subscribes to the `/visualization_marker` topic by default.

# Publishing a Marker

Let's create a new demo file for publishing markers in `applications/scripts/marker_demo.py`.

You should know how to create executable Python nodes on your own by now.
Here is a basic outline of what you need:
```py
#!/usr/bin/env python

import rospy

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
  rospy.init_node('my_node')
  wait_for_time()

if __name__ == '__main__':
  main()
```
Refer to other demos if you get stuck.

Here is what you will need to import into your script:
```py
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
```

Next you will need to create a publisher for `Marker` type messages in your main function, before going into the infinite loop.

```py
marker_publisher = rospy.Publisher('visualization_marker', Marker)
```

Before moving on, it would be a good idea to check what the Marker message involves using some of the ROS command line tools that you know very well by now.

Here is a function you can add to your script for publishing a `Marker` of type `Marker.TEXT_VIEW_FACING` which will include the `text` argument passed to the function.

```py
def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)
```

Notice that when you call this function from your main function, the marker does not show up in RViz.
One way to debug whether this is an issue with our ROS code or with the visualization itself is to see if the message gets published.
To do this, run `rostopic echo /visualization_marker` and try running your script again.
You should observe that no messages are published on this topic, even though your code *does* publish to this topic!

The reason for this is that after you advertise a publisher in your code, you need to wait a little bit for the ROS master to notify other nodes and for those nodes to start subscribing to you.
All of this happens in the background, so you should just wait for a half second or so:
```py
marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
rospy.sleep(0.5)                                                             
show_text_in_rviz(marker_publisher, 'Hello world!')
```

Now run the script, and you should see some green text appear in front of the robot and disappear 1.5 seconds later.
![image](https://cloud.githubusercontent.com/assets/1175286/25159015/7f0c662c-2461-11e7-9602-c5bb754ab600.png)

Next, try modifying the different parameters of the Marker message, re-run the script, and observe the effects on the visualization.
Try to change the following:
- The color
- The size of the text
- The location of the text
- The lifetime before the text disappears (including an infinite lifetime)

**Note:** [rviz/DisplayTypes/Marker](http://wiki.ros.org/rviz/DisplayTypes/Marker) is the best reference for how to fill out a marker message.
Note that some fields are shared between all types of markers, while other fields are used in different ways depending on the marker type.

# Visualizing the robot path with Markers

Next, create a script to publish a Marker that visualizes the path taken by the robot.
To do that you will need to know where the robot is at any given time.
To that end, use ROS command line tools to explore the data in the `/odom` or `/odom_combined` topics.
You will need to extend your script to subscribe to these messages and update parameters of the path visualization based on the messages received on this topic.

To visualize the path you can use `Marker.LINE_STRIP` or `Marker.SPHERE_LIST` type markers.
Rather than adding points to the path with a constant frequency, try adding points only when the robot is displaced by a certain amount from its previous pose. 

When you are done modifying your script, you should be able to move the robot around with the the keyboard teleoperation and observe the trace that it leaves behind.

Hints:
- You can get the current location of the robot from the `/odom` topic.
- You will need to keep track of where you last rendered a point on your path. If you are using the `LINE_STRIP` marker, then you will want to keep around a list of points that the robot has visited. Either way, you will need to store some state and use it with your subscriber callback. The best way to do this is with a class. You can make a subscriber call a class method as follows:

```py
class NavPath(object):
    def __init__(self):
        self._path = []
            
    def callback(self, msg):
        rospy.loginfo(msg)
        if SOME_CONDITION:
            self._path.append(msg.foo.bar)

def main():
    # ...setup stuff...
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()
```

Note that you need to add `rospy.spin()` to the end of your `main` function because this node needs to run forever to monitor the `/odom` topic.

An example using the `SPHERE_LIST` marker type:
![image](https://cloud.githubusercontent.com/assets/1175286/25159798/02bbb46e-2467-11e7-9be3-0ca0c600978d.png)

An example using the `LINE_STRIP` marker type:
![image](https://cloud.githubusercontent.com/assets/1175286/25159872/7ae4b2f6-2467-11e7-8c28-25396779737b.png)