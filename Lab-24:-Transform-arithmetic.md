In [Lab 14](https://github.com/cse481wi18/cse481wi18/wiki/Lab-14%3A-Odometry-and-rotations), we discussed how to interpret rotations.
This lab will expand on that and explain *transformations*, which combine rotation and translation.
This will be useful for computing pre-grasp poses and other waypoints.

# Transformations
A transformation is a combination of a rotation and translation.
As discussed in the previous lab, the notation <sup>A</sup>T<sub>B</sub> means that T describes frame B in terms of frame A.
A 3D rotation matrix <sup>A</sup>R<sub>B</sub> is 3x3, and the columns of the matrix are the standard basis (i.e., the unit X, Y, and Z vectors) of the frame B in terms of frame A.
This assumes that the two frames have the same origin.

A *homogeneous transform matrix* <sup>A</sup>T<sub>B</sub> is 4x4 and describes both the rotation and the translation offset of frame B in terms of frame A.
The rotation matrix is embedded in the upper left hand corner of T.
The translation vector is embedded in the upper right hand corner of T.
The bottom row of a homogeneous transform matrix is always 0, 0, 0, 1.

The translation vector describes the origin of frame B in terms of frame A.

**Example:**
Here is a homogeneous transform matrix <sup>A</sup>T<sub>B</sub>.
What does frame B look like?
```
| cos(45) -sin(45) 0  0   |
| sin(45)  cos(45) 0  0   |
| 0        0       1  0.5 |
| 0        0       0  1   |
```

*Answer:*
The rotation matrix is in the upper left corner.
The rotation matrix tells us that if frame B had the same origin as frame A, frame B would be the same as frame A, but rotated by 45 degrees around the Z axis.
However, B does not have the same origin as A. B's origin, in A's coordinate system, is (0, 0, 0.5).
So B is rotated by 45 degrees and is raised up by 0.5 compared to A.

**Example 2:**
With your right hand, point your index finger forward, your middle finger to the left, and your thumb up.
Your index finger is the X axis, your middle finger is the Y axis, and your thumb is the Z axis.

Now, roll your hand so that your thumb is pointing left and your middle finger is pointing down.
Next, move your hand 10 centimeters closer to you.

What is the homogeneous transform matrix describing your hand's new pose (frame B) in terms of its first pose (frame A)?

*Hints:*
- Remember that the columns of the rotation matrix describing B in terms of A are the unit X, Y, and Z vectors of B in A's coordinate system.

# Pose = transformation
You have worked with `geometry_msgs/Pose` in previous labs, and you know that it has a `position` and `orientation` field.
Because a transformation is a position and orientation, a `geometry_msgs/Pose` can be thought of as a transformation.
However, you often need to specify what this transformation is relative to.
`PoseStamped` messages contain a `header` field, which contains a `frame_id`.
The `frame_id` is equivalent to "frame A" in the examples above.

# Transforming points and poses
Given <sup>A</sup>T<sub>B</sub>, you can take a point, written in terms of frame B (i.e., <sup>B</sup>P), and rewrite the same point in terms of frame A (i.e., <sup>A</sup>P).

<sup>A</sup>P = <sup>A</sup>T<sub>B</sub> * <sup>B</sup>P

Here is a quick sanity check: the point (1, 0, 0) in frame B is the unit X axis of B.

Using the matrix from Example 1:
```
| cos(45) -sin(45) 0  0   |
| sin(45)  cos(45) 0  0   |
| 0        0       1  0.5 |
| 0        0       0  1   |
```

Let's add an extra 1 to the end of <sup>B</sup>P to make it 4-dimensional.
Then <sup>A</sup>T<sub>B</sub> * <sup>B</sup>P is:
```
| cos(45) -sin(45) 0  0   |     | 1 |
| sin(45)  cos(45) 0  0   |  *  | 0 |
| 0        0       1  0.5 |     | 0 |
| 0        0       0  1   |     | 1 |
```

So <sup>A</sup>P is (cos 45, sin 45, 0.5).

# Visualizing with RViz
We will visualize the above example with RViz.

Create a demo called `transformation_demo.py`.
```py
#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg
import rospy
import tf.transformations as tft


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def cosd(degs):
    return math.cos(degs * math.pi / 180)


def sind(degs):
    return math.sin(degs * math.pi / 180)


def axis_marker(pose_stamped):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'axes'
    marker.header = pose_stamped.header
    marker.pose = pose_stamped.pose
    marker.type = visualization_msgs.msg.Marker.LINE_LIST
    marker.scale.x = 0.1

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))
    marker.points.append(Point(1, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))
    marker.points.append(Point(0, 1, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))
    marker.points.append(Point(0, 0, 1))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))

    return marker


def transform_to_pose(matrix):
    pose = Pose()
    # TODO: fill this out
    return pose


def arrow_marker(point):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'arrow'
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.header.frame_id = 'frame_a'
    marker.points.append(Point(0, 0, 0))
    marker.points.append(point)
    marker.scale.x = 0.1
    marker.scale.y = 0.15
    marker.color.r = 1
    marker.color.g = 1
    marker.color.a = 1
    return marker


def main():
    rospy.init_node('transformation_demo')
    wait_for_time()
    viz_pub = rospy.Publisher(
        'visualization_marker', visualization_msgs.msg.Marker, queue_size=1)
    rospy.sleep(0.5)
    b_in_a = np.array([
        [cosd(45), -sind(45), 0, 0],
        [sind(45), cosd(45), 0, 0],
        [0, 0, 1, 0.5],
        [0, 0, 0, 1]
    ])
    ps = PoseStamped()
    ps.header.frame_id = 'frame_a'
    ps.pose = transform_to_pose(b_in_a)
    viz_pub.publish(axis_marker(ps))

    #point_in_b = np.array([1, 0, 0, 1])
    #point_in_a = np.dot(b_in_a, point_in_b)
    #rospy.loginfo(point_in_b)
    #rospy.loginfo(point_in_a)
    #point = Point(point_in_a[0], point_in_a[1], point_in_a[2])
    #viz_pub.publish(arrow_marker(point))

if __name__ == '__main__':
    main()
```

This script will visualize the frame B described by the matrix `b_in_a`.
Open RViz and changed the "Fixed Frame" to `frame_a`.
This will not be in the dropdown list, it is just an imaginary frame of our own making, so just type in the frame name.
Next, add a Marker display on the `visualization_marker` topic and add an `Axes` display with reference frame set to `frame_a`.

Fill out `transform_to_pose` and run the script.
Use `tf.transformations` to help convert between quaternions and matrices:
```py
import tf.transformations as tft
import numpy as np

tft.quaternion_from_matrix(matrix)
tft.quaternion_matrix([x, y, z, w]) # Takes in a plain list of numbers.
```

If you have implemented `transform_to_pose` correctly, you should get this:
![image](https://cloud.githubusercontent.com/assets/1175286/25461997/f3a5d834-2aa0-11e7-9484-1e04c096a3eb.png)

You can see that `b_in_a` describes a frame that is rotated by 45 degrees and raised by 0.5 meters.

Now, uncomment the last part, which implements the equation <sup>A</sup>T<sub>B</sub> * <sup>B</sup>P, and you should see a yellow arrow pointing to frame B's unit X vector:
![image](https://cloud.githubusercontent.com/assets/1175286/25462432/c013994a-2aa3-11e7-8374-0e6bc1c7b8f1.png)

## Transforming poses
Transforming a pose is similar to transforming a point.
Let's say you have the transform describing B in terms of A, and you have a pose that describes C in terms of B.
Then, you can chain transformations together like so:

<sup>A</sup>T<sub>C</sub> = <sup>A</sup>T<sub>B</sub> * <sup>B</sup>T<sub>C</sub>

And this can go on and on:

<sup>A</sup>T<sub>E</sub> = <sup>A</sup>T<sub>B</sub> * <sup>B</sup>T<sub>C</sub> * <sup>C</sup>T<sub>D</sub> * <sup>D</sup>T<sub>E</sub>

# Inverse of a transform
Given <sup>A</sup>T<sub>B</sub>, what is <sup>B</sup>T<sub>A</sub>?

Well, if:

<sup>A</sup>P = <sup>A</sup>T<sub>B</sub> * <sup>B</sup>P

Then we should be able redescribe <sup>A</sup>P in frame B with:

<sup>B</sup>P = <sup>B</sup>T<sub>A</sub> * <sup>A</sup>P

So, substituting back:

<sup>A</sup>P = <sup>A</sup>T<sub>B</sub> * <sup>B</sup>T<sub>A</sub> * <sup>A</sup>P

<sup>A</sup>T<sub>B</sub> * <sup>B</sup>T<sub>A</sub> = I

<sup>B</sup>T<sub>A</sub> = <sup>A</sup>T<sup>-1</sup><sub>B</sub>

**Example:**
Your perception module tells you that there is an object exactly 1 meter in front of the camera frame (`camera_link`).
From TF, you can get `base_link` -> `camera_link` and `base_link` -> `gripper_link`.
Where is the object relative to the gripper?

*Answer*:
We have:

<sup>base_link</sup>T<sub>camera_link</sub>

<sup>base_link</sup>T<sub>gripper_link</sub>

And the position of the object P is in the camera frame:

<sup>camera_link</sup>P

We want <sup>gripper_link</sup>P. We can chain transformations like so, with the `base_link` -> `gripper_link` transform backwards:

<sup>gripper_link</sup>P = <sup>gripper_link</sup>T<sub>base_link</sub> * <sup>base_link</sup>T<sub>camera_link</sub> * <sup>camera_link</sup>P

So we just invert the one backward transform:

<sup>gripper_link</sup>P = <sup>base_link</sup>T<sup>-1</sup><sub>gripper_link</sub> * <sup>base_link</sup>T<sub>camera_link</sub> * <sup>camera_link</sup>P

# Putting it into practice
Let's say your perception module tells you there's an object at this pose in the `base_link` frame:
```
position:
  x: 0.6
  y: -0.1
  z: 0.7
orientation:
  x: 0
  y: 0
  z: 0.38268343
  w: 0.92387953
```

You want to position the robot's gripper, specifically, the `gripper_link` frame, such that it is aligned with the object's X axis and 10 centimeters behind the object (by "behind the object" we mean closer to the robot).
What is the pose (called the pre-grasp pose) that you should send the end-effector to, in the `base_link` frame?

The pose of the object can be thought of as <sup>base_link</sup>T<sub>object</sub>.
We first specify the pre-grasp pose in terms of the object's pose.
The pre-grasp pose is aligned with the object, meaning it has the same orientation.
So relative to the object frame, the pre-grasp frame should have the identity orientation.
A point 10 cm behind the object is <sup>object</sup>P = (-0.1, 0, 0).
This specifies the pre-grasp pose, which you can convert into matrix form: <sup>object</sup>T<sub>pre-grasp</sub>.

Now, you can compute <sup>base_link</sup>T<sub>pre-grasp</sub> and convert it back into a pose message.

Write a script or open a Python interpreter to do these calculations.
Keep in mind that in NumPy, you use `np.dot(A, B)` to multiple matrices or vectors.
`A * B` does element-wise multiplication instead!

You should get this answer:
```
position: 
  x: 0.529289321708
  y: -0.170710677946
  z: 0.7
orientation: 
  x: 0.0
  y: 0.0
  z: 0.382683431234
  w: 0.92387953298
```