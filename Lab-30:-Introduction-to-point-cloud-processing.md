In previous labs, we showed how the robot can detect AR tags placed in the environment.
However, AR tags are not always the best solution, for a variety of reasons:
- The AR tags may not always be visible
- The tag may be larger than the object itself
- It may be impractical, expensive, or ugly to place too many AR tags in the environment

In the next few labs, we will show how to process point cloud data from the robot's head-mounted depth camera.
This will give you the tools you need to write your own perception pipeline.

# Point clouds
The robot's depth sensor works by combining a depth camera and an RGB camera.
The depth camera tells you how far each pixel is from the camera, while the RGB camera tells you the color of that pixel.
These images can be combined into an RGBD (red/green/blue/depth) image, which are often represented as *point clouds*.
A point cloud is simply a list of *points*, where each point represents an X/Y/Z position and, optionally, R/G/B color.

# PCL
The most common point cloud processing framework in ROS is [PCL](http://pointclouds.org/documentation/) (Point Cloud Library).
PCL is fairly well documented:
- [API documentation](http://docs.pointclouds.org/1.7.1/) (we are using version 1.7.1)
- [Tutorials](http://pointclouds.org/documentation/tutorials/)

# Filtering point clouds
In this lab, we will see how to use PCL to filter point clouds using two basic operations: cropping and downsampling.

First, open RViz and display a saved point cloud, ideally with some objects on a tabletop surface.
You can either use a point cloud from a previous lab or save a new one.
```
rosrun perception publish_saved_cloud.py ~/data/tags.bag
```

![image](https://cloud.githubusercontent.com/assets/1175286/25724488/f9882032-30d0-11e7-803b-d52ab4a4e694.png)

# Set up demo code
Create a new executable file in `perception/src/point_cloud_demo.cpp`:
```cpp
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::spin();
  return 0;
}
```

As usual, we need to edit `perception/CMakeLists.txt`.
Modify the commented-out template to look like this:
```cmake
add_executable(perception_point_cloud_demo src/point_cloud_demo.cpp)
set_target_properties(perception_point_cloud_demo PROPERTIES OUTPUT_NAME point_cloud_demo PREFIX "")
add_dependencies(perception_point_cloud_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_point_cloud_demo
  ${catkin_LIBRARIES}
)
```

Test your build.
Your build should succeed and you should be able to run your program, although it does nothing but spin:
```
catkin build
rosrun perception point_cloud_demo # Does nothing
```

## Set up the cropper
Create two files: `src/crop.cpp` and `include/perception/crop.h`:

**`include/perception/crop.h`**
```cpp
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception {
class Cropper {
 public:
  Cropper();
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
};
}  // namespace perception
```

**`src/crop.cpp`**
```cpp
Cropper::Cropper() {}

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  ROS_INFO("Got point cloud");
}
```

Add the cropper library to your CMakeLists.txt.
You can come up with this by copying and modifying the commented-out templates:
```cmake
add_library(perception_crop src/crop.cpp)
add_dependencies(perception_crop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_crop ${catkin_LIBRARIES})
```

Export your cropper library:
```cmake
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    perception_crop
# CATKIN_DEPENDS roscpp rospy
# DEPENDS system_lib
)
```

This will allow other packages to use the cropper library by calling `find_package(catkin REQUIRED COMPONENTS perception)` in their own CMakeLists.txt.

Finally, add the `include` folder to the list of `include_directories`:
```cmake
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

# Reading a point cloud
You can read a point cloud either by subscribing to a `sensor_msgs/PointCloud2` topic, or by using `ros::topic::waitForMessage`.
Since we used `waitForMessage` in the previous lab, we will use the subscriber approach this time.
Update `point_cloud_demo.cpp` to subscribe to a point cloud topic and use `perception::Cropper::Callback` as your callback:

```cpp
#include "perception/crop.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  perception::Cropper cropper;
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  ros::spin();
  return 0;
}
```

Note that we are subscribing to `cloud_in`, as opposed to a known point cloud topic.
We can always remap `cloud_in` to a real topic name later.

Our executable (`point_cloud_demo.cpp`) depends on a library (`crop.h`), so we will need to update CMakeLists.txt again to link the library to the demo:

```cmake
add_executable(perception_point_cloud_demo src/point_cloud_demo.cpp)
set_target_properties(perception_point_cloud_demo PROPERTIES OUTPUT_NAME point_cloud_demo PREFIX "")
add_dependencies(perception_point_cloud_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_point_cloud_demo
  perception_crop
  ${catkin_LIBRARIES}
)
```

Run your demo, remapping `cloud_in` to `mock_point_cloud`.
```
> catkin build
> rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
/point_cloud_demo Callback:15: Got point cloud
/point_cloud_demo Callback:15: Got point cloud
/point_cloud_demo Callback:15: Got point cloud
```

We have just received a `sensor_msgs/PointCloud2` message, but PCL has its own representation of a point cloud, `pcl::PointCloud`.
To convert between the two, we need to use [`pcl_conversions`](http://wiki.ros.org/pcl_conversions).

```cpp
// Add this to your #includes
#include "pcl_conversions/pcl_conversions.h"

// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());
}
```

It can be helpful to use the typedefs shown above.
The "C" in `PointC` and `PointCloudC` stand for "color".
If you choose not to use the typedefs (which you should not place in .h files), then you would have to write:
```cpp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
```

Note that we are not creating a `PointCloudC` object, but rather a shared pointer to a `PointCloud`, `PointCloudC::Ptr`.
This is because many methods implemented in PCL take in shared pointers as arguments.
A shared pointer is a pointer that automatically deletes the memory it manages once the last copy of that shared pointer goes out of scope.
They behave pretty much the same way as regular pointers, and you can use the `*` and `->` operators as normal.
You should always create a point cloud by dynamically allocating memory with a shared pointer, as shown above.

# Cropping
Often, the point cloud from the robot contains areas like the floor or the walls that are not relevant to your task.
You can crop a point cloud to focus your attention on a certain area.
Another benefit of cropping a point cloud is that it reduces the amount of work you have to do, since the list of points gets smaller.

Let's crop off the back wall in the point cloud shown above.
Of course, your point cloud will be different.
By looking at it above, we can estimate how far we should crop the point cloud.

![image](https://cloud.githubusercontent.com/assets/1175286/25726156/b0f290da-30d7-11e7-98c8-ad5af446a9e8.png)

This point cloud is unfortunately somewhat at a diagonal relative to the base frame.
But, it looks like we can crop the point cloud at about 0.9 meters in the X direction.

[Eigen](https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html) is roughly the C++ equivalent of `numpy`.

```cpp
#include "pcl/filters/crop_box.h"

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  Eigen::Vector4f min_pt(0.3, -1, 0.5, 1);
  Eigen::Vector4f max_pt(0.9, 1, 1.5, 1);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(*cropped_cloud);
  ROS_INFO("Cropped to %ld points", cropped_cloud->size());
}
```

```
> rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
/point_cloud_demo Callback:24: Got point cloud with 307200 points
/point_cloud_demo Callback:35: Cropped to 176667 points
```

Now, let's visualize the cropped point cloud.
To do this, we will add a publisher to our demo and use `pcl_conversions` to convert PCL point clouds back to sensor_msgs/PointCloud2 messages.
One technique for incrementally developing and testing point cloud processing pipelines is to publish intermediate results to their own topics.
It often helps if these topics are latched so that you can turn them on and off in RViz.

```cpp
// crop.h
class Cropper {
 public:
  Cropper(const ros::Publisher& pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher pub_;
};

// crop.cpp
Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  ...
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cropped_cloud, msg_out);
  pub_.publish(msg_out);
}

// point_cloud_demo.cpp
int main(int argc, char** argv) {
  ...
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  Cropper cropper(crop_pub);
  ...
}
```

Now visualize your crop in RViz.
Here, we add a display for the cropped cloud topic, `cropped_cloud`, and change its color to AxisColor with alpha set to 0.05.
![image](https://cloud.githubusercontent.com/assets/1175286/25727057/fec1a158-30db-11e7-846a-fd9948162ce1.png)

Oops.
This crop doesn't quite match.

It will get very tedious to have to recompile our program to change hard-coded values.
Instead, we should read them from the parameter server:

```cpp
double min_x, min_y, min_z, max_x, max_y, max_z;
ros::param::param("crop_min_x", min_x, 0.3);
ros::param::param("crop_min_y", min_y, -1.0);
ros::param::param("crop_min_z", min_z, 0.5);
ros::param::param("crop_max_x", max_x, 0.9);
ros::param::param("crop_max_y", max_y, 1.0);
ros::param::param("crop_max_z", max_z, 1.5);
Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
```

We can update the parameter values at runtime:
```
rosparam set crop_max_x 0.86
```

You should see the visualization of your crop immediately update as you change the parameter values.

![image](https://cloud.githubusercontent.com/assets/1175286/25727139/76026798-30dc-11e7-8671-db1199b928b8.png)

Other point cloud operations work in the same way as `CropBox`.
Usually, you call `setInputCloud` to set the input to the algorithm.
Then, you supply some algorithm-specific parameters.
Finally, the output is given through an algorithm-specific method, e.g., `filter`, `segment`, `align`, etc.

To figure out the exact way to use one of these algorithms, go to the [PCL API documentation](http://docs.pointclouds.org/1.7.1/) and search for the algorithm name (e.g., "CropBox").
Or, just do a web search for what you want to do.

# Downsampling
Downsampling is sometimes useful for complex operations on large point clouds.
It allows you to sample the data so that there isn't so much of it.
Downsampling is also useful for streaming the point cloud data from the real robot.
As we have seen, the robot's wireless connection can be a bit slow, and so it might help to stream cropped, downsampled data instead.

Here is how to downsample a point cloud using a VoxelGrid filter:
```cpp
#include "pcl/filters/voxel_grid.h"

PointCloudC::Ptr downsampled_cloud(new PointCloudC());
pcl::VoxelGrid<PointC> vox;
vox.setInputCloud(cloud);
double voxel_size;
ros::param::param("voxel_size", voxel_size, 0.01);
vox.setLeafSize(voxel_size, voxel_size, voxel_size);
vox.filter(*downsampled_cloud);
```

Create a class called `Downsampler` that, like `Cropper`, has a callback that downsamples a point cloud and publishes the point cloud.
To create this class, you will need to go through the same process that we used to create the Cropper class:
- Create downsample.h and downsample.cpp files
- Add the library to CMakeLists.txt
- Export your library to other packages
- Link the library with the `point_cloud_demo` executable.

Set up your `Downsampler` so that it subscribes to the `cropped_cloud` topic.
This sets up a pipeline in which the image is first cropped, then downsampled.

Note that you don't have to set up a pipeline this way.
A more straightforward way to do this is to just put the code in sequence:
```cpp
pcl::CropBox crop;
crop.setInputCloud(input);
...
crop.filter(*cropped_cloud);

pcl::VoxelGrid vox;
vox.setInputCloud(cropped_cloud);
vox.filter(*output);
```

Here is a somewhat busy visualization that shows the crop as a partially transparent rainbow, and the downsampled points as red dots.
You can adjust the RViz visualization to your liking:

![image](https://cloud.githubusercontent.com/assets/1175286/25727526/928713bc-30de-11e7-8e3c-9a3fcb8cabe0.png)

# Min, max, and center of a point cloud
The last part of this intro will show how to find the min, max, and center of a point cloud.

A [`PointCloud`](http://docs.pointclouds.org/1.7.1/classpcl_1_1_point_cloud.html) has a member called `points`, which you can use to iterate through the list of points in the point cloud.
You can easily compute the minimum and maximum X, Y, and Z values by iterating through the points:
```cpp
void GetMinMax(PointCloudC::Ptr input, double* min_x, double* max_x) {
  *min_x = std::numeric_limits<double>::max();
  *max_x = std::numeric_limits<double>::min();
  for (size_t i = 0; i < input->size(); ++i) {
    const PointC& pt = input->at(i);
    if (pt.x < *min_x) {
      *min_x = pt.x;
    }
    if (pt.x > *max_x) {
      *max_x = pt.x;
    }
  }
}
```

This tedious code has already been written for you in [`pcl::getMinMax3D`](http://docs.pointclouds.org/1.7.1/group__common.html#ga3166f09aafd659f69dc75e63f5e10f81):
```cpp
#include "pcl/common/common.h"

PointC min_pcl;
PointC max_pcl;
pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);
```

This will be useful for visualizing your output.
You can publish a box marker around a detected object, for example, by computing its min, max, and center points (where the center point is just (max + min) / 2).