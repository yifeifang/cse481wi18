In previous labs, you have programmed the robot to grasp an object, but you had to supply the location of the object through an interface.
Robot perception allows the robot to understand the environment, including the locations of objects, automatically.
In this lab, we will use AR tags, which can be easily spotted and uniquely identified by the robot.
AR tags are easy to use, but requiring too many AR tags to be put in the environment can make for a less appealing product.
![image](http://wiki.ros.org/ar_track_alvar?action=AttachFile&do=get&target=artags.png)

# Recording perception data
We have distributed AR tags randomly to each group.
Take turns with other teams to set up your markers on the shelf and record a point cloud.
Feel free to stick one of the tags on the wall with blue tape.
Also, leave one where you think the robot's arm could feasibly reach (this will be useful later).

Use RViz to make sure you can see all three, then run:
```
rosrun perception save_cloud tags
rosbag info tags.bag
mkdir ~/data
mv tags.bag ~/data
```

# Hallucinate the data
Now that we have saved a point cloud, let's hallucinate it in the scene.
This workflow will allow us to spend most of our time working in simulation but with real sensor data.

## Write the hallucinator
We will write a class in `perception/src/perception/mock_camera.py` that allows us to publish a saved point cloud.
```py
camera = perception.MockCamera()
cloud = camera.read_cloud('~/data/tags.bag')
```

Look at the [rosbag code API](http://wiki.ros.org/rosbag/Code%20API) and figure out how to read the point cloud from the bag file you saved:

```py
class MockCamera(object): 
    """A MockCamera reads saved point clouds.
    """
    def __init__(self):
        pass

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.
    
        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """
        return None
```

Now let's set up the `perception` Python module by creating `perception/src/perception/__init__.py`:
```py
from .mock_camera import MockCamera
```

Create a `setup.py` for your module in `~/catkin_ws/src/cse481c/perception/setup.py`
```py
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['perception'],
    package_dir={'': 'src'})

setup(**setup_args)
```

Finally, in `perception/CMakeLists.txt`, uncomment `catkin_python_setup()`.

Now, run `catkin build`.

Add a demo to `applications` that just reads a point cloud and republishes it:
```py
#!/usr/bin/env python 

from sensor_msgs.msg import PointCloud2
import perception
import rospy


def wait_for_time(): 
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

    
def main():                                                                             
    rospy.init_node('publish_saved_cloud')
    wait_for_time()                                                                     
    argv = rospy.myargv()
    if len(argv) < 2:
        print 'Publishes a saved point cloud to a latched topic.'
        print 'Usage: rosrun applications publish_saved_cloud.py ~/cloud.bag'
        return
    path = argv[1]
    camera = perception.MockCamera()
    cloud = camera.read_cloud(path)

    if cloud is None:
        rospy.logerr('Could not load point cloud from {}'.format(path))
        return

    pub = rospy.Publisher('mock_point_cloud', PointCloud2, queue_size=1)       
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        cloud.header.stamp = rospy.Time.now()
        pub.publish(cloud)
        rate.sleep()                                          
    
    
if __name__ == '__main__':
    main()
```

Run your code and check RViz to see your point cloud.
![image](https://cloud.githubusercontent.com/assets/1175286/25605042/fd172060-2ebc-11e7-83ae-cad69ccc8c4b.png)

Note that this node just reads the saved data and continually publishes it.
But you can do more.
For example, suppose you have code like:
```
Go to Kitchen
Look for soda can
Pick up soda can
Go to Table
Place soda can on Table
```

You can save two scenes: one that the robot should see in the Kitchen, and one at the Table.
Then, you can load the mock point cloud after arriving at each location:
```py
if is_sim:
  cloud = camera.read_cloud('kitchen.bag')
else:
  cloud = rospy.wait_for_message('head_camera/depth_registered/points', PointCloud2)
```

You can additionally transform the pre-recorded cloud to be at a particular location relative to the map or to the robot.
You can visualize how the robot moves relative to the hallucinated data to get an idea of whether your code is correct.
Once you have built some confidence in your system, you can test on the real robot.

# ar_track_alvar
We will use the [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) package to track AR tags.
We have already printed out some tags for you to use.

## Install
The `ar_track_alvar` package has not been installed on the course computers.
To install a ROS package named `package_name`, run `sudo apt-get install ros-indigo-package-name`.
Install `ar_track_alvar` now.

**Note:** The AR tag package installed on the Fetch is an old version that has a different API.
For now, we will use the newer package.

## Configure for the Fetch
Read the documentation for `ar_track_alvar`.
Section 4.1 explains that you can detect markers by running the `individualMarkers` node.
It also explains various arguments that you can pass in.

Rather than set all of the arguments on the command line, you should create a launch file.
The package provides a launch file to run on a PR2 robot.
You can copy this launch file and edit it to configure `ar_track_alvar` to run on your computer for the simulated Fetch.
```
roscp ar_track_alvar pr2_indiv.launch ~/catkin_ws/src/fetch_api/launch/ar_desktop.launch
```

Edit the launch file and make the following changes:
- The `marker_size` is 4.5 cm, not 4.4
- Set `cam_image_topic` to `/head_camera/depth_registered/points`.
- Set `cam_info_topic` to `/head_camera/rgb/camera_info`
- Change the `output_frame` to `base_link`.

## Look for AR tags
Now, you can finally run the AR tracker on your pre-recorded data!
Note that we need to set a different value of `cam_image_topic` so that the AR tag tracker will subscribe to our mock data instead of the simulated depth camera.

```
roslaunch fetch_api ar_desktop.launch cam_image_topic:=mock_point_cloud
```

## Visualize the output
According to the documentation, `individualMarkers` publishes a visualization of the markers to `visualization_marker`.
In the future, you may want to remap this topic to something else.
For now, you can visualize the output by running RViz and adding a marker display.

## Access the data programmatically
To actually read the poses of the AR tags, you can subscribe to the `ar_pose_marker` topic:
```
rostopic echo /ar_pose_marker
```

# Reaching for a hallucinated marker
As the last step of this lab, let's program the robot to reach for one of the markers in simulation.

Create a demo called `hallucinated_reach.py` in which the robot reaches the AR marker locations and reaches for one of them.

Hints:
- Run `rostopic echo /ar_pose_marker` to get a sample of the data. Pay close attention to the headers and frame_ids.
- arm.move_to_pose commands a position for the `wrist_roll_link`, not the `gripper_link`. Therefore, the wrist should move over one of the markers. It's okay for your gripper to collide with the scene in this lab because we are just demonstrating the functionality of the system. It will be up to you to put together a working system for your projects :).
- Be sure to run MoveIt before running this code
- Raise the torso to full height before starting
- It is normal for some markers to not be found. If none of them are found (or none of them are reachable), try setting up your tags differently and recording more point clouds.

```py
#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped
import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = fetch_api.Arm()
    arm.move_to_pose(start)
                                                                               
    reader = ArTagReader()
    sub = rospy.Subscriber(...) # Subscribe to AR tag poses, use reader.callback
    
    while len(reader.markers) == 0:
        rospy.sleep(0.1)
    
    for marker in reader.markers:
        # TODO: get the pose to move to
        error = arm.move_to_pose(???)
        if error is None:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            return
        else:
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))
    rospy.logerr('Failed to move to any markers!')


if __name__ == '__main__':
    main()
```