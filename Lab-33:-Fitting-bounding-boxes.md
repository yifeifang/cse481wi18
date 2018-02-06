If you are interested in fitting a better bounding box (rather than use an axis-aligned box), this lab is for you.
Fetch has created a package called [`simple_grasping`](http://wiki.ros.org/simple_grasping) which includes a shape fitter.

Install `simple_grasping` if you don't already have it:
```
sudo apt-get install ros-indigo-simple-grasping
```

`simple_grasping` has a function called [`extractShape`](http://docs.ros.org/indigo/api/simple_grasping/html/namespacesimple__grasping.html#a5e500982f834eea5d24a33c8c63df2fe), which will fit a bounding box as snugly as possible around a given point cloud.

To do this, you will need to output the `ModelCoefficients` from `SegmentSurface`:
```
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices::Ptr indices,
                    pcl::ModelCoefficients::Ptr coeff);
```

This is a relatively simple modification, just remove the local variable and replace it with the passed in `coeff` argument.

Then, when visualizing the table, use `simple_grasping::extractShape` instead of `GetAxisAlignedBoundingBox`.

```cpp
#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

PointCloudC::Ptr extract_out(new PointCloudC());
shape_msgs::SolidPrimitive shape;
geometry_msgs::Pose table_pose;
simple_grasping::extractShape(*cloud_out, coeff, *extract_out, shape,
                              table_pose);
```

`extractShape` will attempt to match a box and a cylinder to the point cloud.
Make sure it is a box before looking at its dimensions:
```cpp
if (shape.type == shape_msgs::SolidPrimitive::BOX) {
  // scale.x = shape.dimensions[0]
  // scale.y = shape.dimensions[1]
  // scale.z = shape.dimensions[2]
}
```

The box height might be shifted up by 1, so shift it back down if the visualization looks wrong.
Note that this only works if the table is approximately level:
```cpp
table_marker.pose.position-z -= table_marker.scale.z
```

With the uncropped point cloud, the table segmentation visualization now looks like this:
![image](https://cloud.githubusercontent.com/assets/1175286/25833953/b5ef990c-3429-11e7-89c7-4aef8da92845.png)

Currently, this only affects the visualization.
The orientation of the table is not used when segmenting the objects above it.
However, it could lead to a more accurate bounding box if you choose to use this code to add obstacles to the MoveIt planning scene.