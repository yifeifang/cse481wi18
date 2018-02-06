In this lab, we will see how to analyze point clouds to understand where objects are in the environment.
As we have seen, point clouds are just a list of XYZRGB points.
*Segmentation* is the process of categorizing the points into semantic groups.
For example, we will figure out which points represent the tabletop or shelf, and which points represent the objects resting on the surface.

# Tabletop segmentation
First, we will see how to segment a horizontal surface from the scene.

## Setup
Create `src/segmentation.cpp` and `include/perception/segmentation.h`.
Add a `perception_segmentation` library to your `CMakeLists.txt` as we did in the previous tutorials.

**include/perception/segmentation.h**
```cpp
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception {
// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices::Ptr indices);

class Segmenter {
 public:
  Segmenter(const ros::Publisher& surface_points_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher surface_points_pub_;
};
}  // namespace perception
```

**src/segmentation.cpp**
```cpp
#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub)
    : surface_points_pub_(surface_points_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, table_inliers);
}
}  // namespace perception
```

Edit your `point_cloud_demo.cpp` to use the segmenter:

**src/point_cloud_demo.cpp**
```cpp
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  perception::Segmenter segmenter(table_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter);
  ros::spin();
  return 0;
}
```

Pause here and make sure that your code compiles.

## Do the segmentation
To do the segmentation, we will use an algorithm known as [Random Sample Consensus](https://en.wikipedia.org/wiki/Random_sample_consensus), or RANSAC, to fit a plane to the scene.
RANSAC will randomly sample points and attempt to fit a plane through those points.
Then, it checks to see how many points are "close" to the plane (*inliers*) and how many are not (*outliers*).
If a plane model has enough inliers, then the algorithm is done.
Otherwise, it repeats this process.

We will follow [this tutorial](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation) on planar segmentation from the PCL website.

PCL implements the RANSAC algorithm, which can be used in the following way:
```cpp
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

pcl::PointIndices indices_internal;
pcl::SACSegmentation<PointC> seg;
seg.setOptimizeCoefficients(true);
// Search for a plane perpendicular to some axis (specified below).
seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
// Set the distance to the plane for a point to be an inlier.
seg.setDistanceThreshold(0.01);
seg.setInputCloud(cloud);

// Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
Eigen::Vector3f axis;
axis << 0, 0, 1;
seg.setAxis(axis);
seg.setEpsAngle(pcl::deg2rad(10.0));

// coeff contains the coefficients of the plane:
// ax + by + cz + d = 0
pcl::ModelCoefficients coeff;
seg.segment(indices_internal, coeff);

*indices = indices_internal;

if (indices->indices.size() == 0) {
  ROS_ERROR("Unable to find surface.");
  return;
}
```

Add this to `SegmentSurface`.

## PointIndices
A new data structure we are using is [`PointIndices`](http://docs.pointclouds.org/1.7.1/structpcl_1_1_point_indices.html).
`PointIndices` are used to represent a subset of a point cloud, using a list of indices in the `PointCloud`'s `points` field.
In the case of planar segmentation, the indices represent the points of the cloud corresponding to the planar surface.
Some PCL algorithms make use of `PointIndices`, while others don't.
`PointIndices` is a wrapper around a `std::vector<int>`, which you can get using the `indices` field, like so:
```cpp
PointCloudC::Ptr cloud(new PointCloudC);
pcl::PointIndices indices;
for (size_t i=0; i<indices.indices.size(); ++i) {
  int index = indices.indices[i];
  const PointC& pt = cloud->points[index];
}
```

To [reify](https://www.google.com/search?q=define+reify) a point cloud from its indices, use [`pcl::ExtractIndices`](http://docs.pointclouds.org/1.7.1/classpcl_1_1_extract_indices.html):
```cpp
#include "pcl/filters/extract_indices.h"

// Given these data types:
PointCloudC::Ptr original_cloud(new PointCloudC);
pcl::PointIndices indices;
PointCloudC::Ptr subset_cloud(new PointCloudC);

// Extract subset of original_cloud into subset_cloud:
pcl::ExtractIndices<PointC> extract;
extract.setInputCloud(original_cloud);
extract.setIndices(indices);
extract.filter(*subset_cloud);
```

# Visualize the segmented point cloud
The `Segmenter::Callback` calls `SegmentSurface`, which gives you the indices corresponding to the surface.
Reify the point cloud, then convert it to a `sensor_msgs::PointCloud2` (see previous lab), then publish it using `surface_points_pub_`.

If you add a PointCloud2 display in RViz with the `table_cloud` topic, you should see the segmented point cloud:

![image](https://cloud.githubusercontent.com/assets/1175286/25825555/3a804112-33f8-11e7-9578-d50417da59a4.png)

If you look carefully, you will see that the segmented surface includes parts of the objects above the surface.
We will refine this in a later step, but keep in mind that the segmentation will not always be perfect.

## Compute an axis-aligned bounding box

Let's now visualize a box around the surface.
Adding visualizations will help you debug your code and build trust in your system.
Additionally, the same technique can be used to add a collision object to your MoveIt PlanningScene (use topics or services to communicate between your C++ and Python code).

A function with this signature to `segmentation.h` and `segmentation.cpp`.
Like `SegmentSurface` it does not have to be a method of `Segmenter`.
In fact, it's more reusable if it's not:
```cpp
// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);
```

In your `point_cloud_demo.cpp`, create a marker publisher and pass it throughout your `Segmenter` class:
```cpp
ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
perception::Segmenter segmenter(table_pub, marker_pub);
```

Implement `GetAxisAlignedBoundingBox` so that it sets the pose and dimensions of a box surrounding the given point cloud.
Because we are computing an *axis-aligned* bounding box, the orientation of the box is just the identity orientation.
Making a box that fits more tightly to the data is a refinement that requires extra work.

Implementation hints:
- Use `pcl::getMinMax3D` as discussed in the previous lab.
- The center X position is (max.x + min.x) / 2.
- The X dimension is max.x - min.x.
- You can't return multiple values from a C++ function, but you can pass in pointers and have the function mutate those pointers. Set pointer values like so: `pose->orientation.w = 1`.

Once you have implemented `GetAxisAlignedBoundingBox`, you can create a marker and publish it:
```cpp
visualization_msgs::Marker table_marker;
table_marker.ns = "table";
table_marker.header.frame_id = "base_link";
table_marker.type = visualization_msgs::Marker::CUBE;
GetAxisAlignedBoundingBox(table_cloud, &table_marker.pose, &table_marker.scale);
table_marker.color.r = 1;
table_marker.color.a = 0.8;
marker_pub_.publish(table_marker);
```

![image](https://cloud.githubusercontent.com/assets/1175286/25832316/a4f80950-341e-11e7-9567-1a5ec6a09025.png)

## Using ModelCoefficients
The planar segmentation we did assumes that points 0.01 meters above or below the plane are part of the plane.
You can trim this value by changing the value passed to `setDistanceThreshold`.
However, you may be interested in mostly trimming the points above the plane, while keeping the points below the plane.

The `ModelCoefficients` computed by the segmentation give you the a, b, c, and d values of the plane, where the plane is defined by:

```
ax + by + cz + d = 0
```

For a perfectly horizontal plane at a height of 1.2 meters (z=1.2), the coefficients would be:
```
(0)x + (0)y + (1)z - 1.2 = 0
0, 0, 1, 1.2
```

So to compute which points are above the plane and which are below, just plug in values of x, y, and z in `SegmentSurface` and see if the value is above or below 0.
Due to imperfections in the plane fitting, we actually check if the point is slightly above the plane:

```cpp
double distance_above_plane;
ros::param::param("distance_above_plane", distance_above_plane, 0.005);

// Build custom indices that ignores points above the plane.
for (size_t i = 0; i < cloud->size(); ++i) {
  const PointC& pt = cloud->points[i];
  float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
              coeff.values[2] * pt.z + coeff.values[3];
  if (val <= distance_above_plane) {
    indices->indices.push_back(i);
  }
}

// Comment this out
//*indices = indices_internal;
if (indices->indices.size() == 0) {
  ROS_ERROR("Unable to find surface.");
  return;
}
```

`distance_above_plane = 0.005`.
It looks like the table includes too much of the objects, but this is because we are visualizing the table as an axis-aligned bounding box, as opposed to a box oriented with the plane.
In the next lab, we will see how to properly visualize the point cloud above the surface.
![image](https://cloud.githubusercontent.com/assets/1175286/25832339/d20b24ea-341e-11e7-84df-33644b80d30b.png)