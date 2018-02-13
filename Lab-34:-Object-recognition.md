Once you have segmented the objects in the scene, you still don't necessarily know which objects are which.
Object *recognition* is the process of identifying objects from perceptual data.

In this lab, we will learn about a two simple approaches to object recognition.
Keep in mind that object recognition is still an active area of research.

# Overview
Two simple ways to distinguish objects from each other are size and color.
In general, different objects will have different bounding box dimensions.
You would be surprised by how well this simple representation works.

However, you might want to distinguish between two different colors of the same object.
Or, maybe an object is partially blocked by another object, so its bounding box size is smaller.
In both these cases, size can't help us.
Another way to represent an object is by its color.
Most objects have more than one color, so we will represent an object by a histogram of its colors.

In both cases, we are reducing an object to a set of *features*, which we can represent as a list of numbers.
In the size representation, an object is summarized using 3 numbers, and we hope that similar objects will have similar numbers.

Colors are a bit trickier.
A color can be represented using RGB values, with 256 values for each of R, G, and B.
Subtle changes in lighting can affect these values, so we should reduce the color space down.
To do this, we can round each of the R, G, and B values down to just five supervalues, with each supervalue spanning (256/5=51) values.
For example, (0, 0, 0) and (45, 45, 45) will be considered the same color, but (0, 0, 0) and (53, 0, 0) will be different colors.
One way to represent the overall color of an object is to just compute its average color.
However, sometimes objects are multi-colored, and their average colors are an ambiguous grey.
In that case, it may be better to represent a color as a distribution over the 5 * 5 * 5 = 125 colors.

To recognize objects, we will first label a dataset of objects and extract features from them.
Then, in a new scene, we will extract the same features from the objects in the scene and see which of the labeled objects matches the most closely.

# Step 1: Download the dataset
First, we need to gather a dataset of objects that we can label.
To do this, we placed a set of objects on a table by itself and recorded a point cloud using the `save_cloud` utility.
We also created a test scene with all of the objects on the table (objects.bag):
![image](https://cloud.githubusercontent.com/assets/1175286/26232097/70e7c42e-3c08-11e7-89d2-ec0bea2343b5.png)

- [Download the dataset](https://drive.google.com/open?id=0B77PnOCaAq8sUHVSVUpyQ0lBRW8)
- Extract the zip file `unzip objects.zip`

# Step 2: Refactor the segmentation pipeline
In this step, we will refactor the segmentation pipeline so that it can be more easily reused.

## Replace simple_grasping with our code
We have modified the `simple_grasping` code from Lab 33 so that it only fits boxes to objects, not cylinders.
This speeds it up and simplifies the output.
The modified code is in `box_fitter.cpp` and `box_fitter.h`, which you can get from the course repository.
`extractShape` has been renamed to `FitBox`.

Update your code so that it uses `FitBox` instead of `simple_grasping::extractShape`.
Then, make some more edits:
- Fit a bounding box using `FitBox` around each of the objects.
- Remove the code that visualizes the table. We have plenty of ways to debug the table extraction now.
- Make sure that your segmenter is getting a cropped point cloud from the cropper.

Instructions for CMakeLists.txt and package.xml
- For some reason, you should not delete `simple_grasping` from your CMakeLists.txt or package.xml. At least, when we tried, we would get the error: `ERROR: qhull was unable to compute a convex hull` unless we continued to link with `simple_grasping`. You only need to leave simple_grasping in your CMakeLists.txt and package.xml, but you can delete all uses of it from your code.
- However, you will need to link the perception_segmentation library with perception_box_fitter.

## Tune parameters for objects.bag
Run the segmentation pipeline on objects.bag.
We found that for the scene `objects.bag`, the following parameters worked well:
```
rosparam set ec_cluster_tolerance 0.03
rosparam set ec_min_cluster_size 700
rosparam set ec_max_cluster_size 7500
rosparam set distance_above_plane 0.01
rosparam set crop_min_x -0.5
rosparam set crop_max_z 1.5
rosparam set crop_max_x 1.05
rosparam set crop_min_z 0.7
```

It can be helpful to store these values in a launch file, since parameters are forgotten every time the ROS Master shuts down:
```xml
<launch>
  <param name="ec_cluster_tolerance" value="0.03" />
  <param name="ec_min_cluster_size" value="700" />
  <param name="ec_max_cluster_size" value="7500" />
  <param name="distance_above_plane" value="0.01" />
  <param name="crop_min_x" value="-0.5" />
  <param name="crop_max_z" value="1.5" />
  <param name="crop_max_x" value="1.05" />
  <param name="crop_min_z" value="0.7" />
  <node pkg="perception" type="point_cloud_demo" name="point_cloud_demo" output="screen">
    <remap from="cloud_in" to="mock_point_cloud" />
  </node>
</launch>
```

Here is what our segmentation output looks like for `objects.bag`:
![image](https://cloud.githubusercontent.com/assets/1175286/26231825/8e99bbc8-3c06-11e7-9c9d-eb9973990c59.png)

Since we tuned the parameters to work with all of the objects on the table, we should be able to extract a single object placed on a table.
Test out all of the different objects (`blue_cup.bag`, etc.) and make sure that the segmenter is segmenting each object correctly.
In particular, check that the cups are detected as a single object:

![image](https://cloud.githubusercontent.com/assets/1175286/26231910/222852f0-3c07-11e7-8f9c-256b1ee62b86.png)

## Refactor the segmentation code
At this point, the segmenter callback takes in a point cloud and figures out the object point clouds and poses.
This is a useful primitive that we will want to reuse in the next steps.
Add a function that takes in a point cloud and outputs a vector of objects.

First, create the file `perception/include/perception/object.h`
```cpp
#ifndef _PERCEPTION_OBJECT_H_
#define _PERCEPTION_OBJECT_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace perception {
struct Object {
  string name;
  double confidence;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 dimensions;
};
}  // namespace perception

#endif  // _PERCEPTION_OBJECT_H_
```

Now add the new function to `segmentation.h`
```cpp
#include "perception/object.h"

// Does a complete tabletop segmentation pipeline.
//
// Args:
//  cloud: The point cloud with the surface and the objects above it.
//  objects: The output objects.
void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects);
```

Implement `SegmentTabletopScene` by copying most of your code from the Segmenter callback.
Then, delete the visualization code.

```cpp
#include "perception/object.h"

void SegmentTabletopScene(PointCloudC::Ptr cloud,
                          std::vector<Object>* objects) {
  // Same as callback, but with visualization code removed.
}
```

Your callback in `segmentation.cpp` should reduce to this:
```cpp
void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud_unfiltered);
  PointCloudC::Ptr cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

  std::vector<Object> objects;
  SegmentTabletopScene(cloud, &objects);

  for (size_t i = 0; i < objects.size(); ++i) {
    const Object& object = objects[i];

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.pose = object.pose;
    object_marker.scale = object.dimensions;
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub_.publish(object_marker);
  }
}
```

Check that your segmenter works as expected.

# Step 3: Extract features

Now, we will extract features from each of the objects.

## Create ObjectFeatures.msg
We will represent the features with an ObjectFeatures message:
```
string object_name # Name of the object
string[] names # Names of each of the features of the object
float64[] values # Values of each feature
```

Usually, in ROS, whenever we want to create a message, we do so in a package that only holds messages.
This enables other packages to use our ObjectFeatures message without having to compile the entire `perception` package.

```
cd cse481c
catkin create pkg perception_msgs --catkin-deps message_generation message_runtime
```

Create `ObjectFeatures.msg` in the `msg` folder of `perception_msgs`:
```
cd perception_msgs
mkdir msg
vim msg/ObjectFeatures.msg
```

Follow the instructions in `perception_msgs/CMakeLists.txt` to see how to generate your message.
You will need to edit `perception_msgs/CMakeLists.txt` so that it looks like the following:
```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  ObjectFeatures.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
```

## Create the labeling tool
We will create a tool that takes in one of the point clouds and the name of the object, and saves an ObjectFeatures to a rosbag:
```
Sample usage:
> rosrun perception extract_features blue_cup.bag blue_cup
> ls
> blue_cup.bag blue_cup_label.bag
```

The tool will populate the ObjectFeatures message with the name of the object as well as the features of the object.

Create a file, `src/extract_features_main.cpp` with the following contents:
```cpp
#include <iostream>
#include <string>
#include <vector>

#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

#include "perception/feature_extraction.h"
#include "perception/object.h"
#include "perception/segmentation.h"
#include "perception/typedefs.h"
#include "perception_msgs/ObjectFeatures.h"

void Crop(PointCloudC::Ptr cloud_in, PointCloudC::Ptr cloud_out) {
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.3);
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 0.9);
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud_in);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(*cloud_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_features");
  ros::Time::init();
  if (argc < 3) {
    std::cout << "Extracts features from a bag file with a point cloud with a "
                 "single object on a table. The features are saved to LABEL.bag"
              << std::endl;
    std::cout << "Usage: rosrun perception extract_features FILE.bag LABEL"
              << std::endl;
    return 0;
  }
  std::string path(argv[1]);
  std::string label(argv[2]);

  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("head_camera/depth_registered/points");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  if (view.size() == 0) {
    std::cerr << "No messages on topic head_camera/depth_registered/points"
              << std::endl;
  }

  PointCloudC::Ptr pcl_cloud(new PointCloudC());
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    sensor_msgs::PointCloud2::ConstPtr cloud =
        it->instantiate<sensor_msgs::PointCloud2>();
    if (cloud == NULL) {
      std::cerr << "Unable to instantiate point cloud." << std::endl;
      return 1;
    }
    pcl::fromROSMsg(*cloud, *pcl_cloud);
    break;
  }

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  Crop(pcl_cloud, cropped_cloud);

  std::vector<perception::Object> objects;
  perception::SegmentTabletopScene(cropped_cloud, &objects);
  if (objects.size() != 1) {
    std::cerr << "Expected to see exactly one object, found " << objects.size()
              << std::endl;
    return 1;
  }

  const perception::Object& object = objects[0];
  perception_msgs::ObjectFeatures features;
  features.object_name = label;
  perception::ExtractSizeFeatures(object, &features);

  rosbag::Bag bag_out;
  bag_out.open(label + "_label.bag", rosbag::bagmode::Write);
  bag_out.write("object_features", ros::Time::now(), features);
  bag_out.close();

  return 0;
}
```

Now, we will implement the feature extraction itself.
We will start by just implementing the size features.
A rectangular object may be rotated and have its x and y values flipped, so instead of storing the x, y, and z dimensions of the bounding box directly, we will define the "box_size_x" feature to be the smaller of the x and y dimensions, while "box_size_y" feature is the larger of the two.
The "box_size_z" feature will just be the z dimension of the object's bounding box.

Add `include/perception/feature_extraction.h":
```cpp
#ifndef _PERCEPTION_FEATURE_EXTRACTION_H_
#define _PERCEPTION_FEATURE_EXTRACTION_H_

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace perception {
void ExtractSizeFeatures(const Object& object,
                         perception_msgs::ObjectFeatures* features);
}  // namespace perception

#endif  // _PERCEPTION_FEATURE_EXTRACTION_H_
```

Now implement `ExtractSizeFeatures` in `src/feature_extraction.cpp`:
```cpp
#include "perception/feature_extraction.h"

#include <algorithm> // std::min and std::max

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace perception {
void ExtractSizeFeatures(const Object& object,
                         perception_msgs::ObjectFeatures* features) {
  // "x" dimension is always the smallest of x and y to account for rotations.
  // z always points up.
  features->names.push_back("box_dim_x");
  features->values.push_back(???);
  features->names.push_back("box_dim_y");
  features->values.push_back(???);
  features->names.push_back("box_dim_z");
  features->values.push_back(???);
}
}  // namespace perception
```

Configure your CMakeLists.txt appropriately and compile.

## Run the feature extractor
Now, you should be able to run your feature extractor.
Make sure that same parameters are loaded as for your point cloud demo (e.g., by running `point_cloud_demo.launch` first).
```
cd ~/data/objects
rosrun perception extract_features blue_cup.bag blue_cup
```

A file called `blue_cup_label.bag` should appear in the directory.
You can inspect the features by subscribing to the `object_features` topic and playing the bag file:
```
rostopic echo object_features
rosbag play blue_cup_label.bag
```

If everything is working, then go ahead and extract features for all of the objects.
```
rosrun perception extract_features expo.bag expo
rosrun perception extract_features galaxy_nexus.bag galaxy_nexus
rosrun perception extract_features orange_dog_toy.bag orange_dog_toy
rosrun perception extract_features red_cup.bag red_cup
rosrun perception extract_features turtle.bag turtle
rosrun perception extract_features yellow_dog_toy.bag yellow_dog_toy
```

# Step 4: Match features

Now, we will write a class that recognizes objects given a labeled dataset.
To do this, we will use a simple matching strategy: nearest neighbor search.
You can think of the vector of feature values as being a point in space.
To recognize a new object, we extract features from that object and look for the nearest labeled point in terms of Euclidean distance.
Although there are efficient data structures for doing nearest neighbor lookups (e.g., KD-trees), we will simply scan through the dataset to find the nearest neighbor.
This is fine for now because we are dealing with a very small number of labeled points.

## Move all of your labeled data
We will read the labeled data from a single folder, so you should move them now:
```
cd ~/data/objects
mkdir labels
mv *_label.bag labels
```

## Create the recognizer
Create two files, `src/object_recognizer.cpp` and `include/perception/object_recognizer.h`.
We have implemented most of the code for you, you just need to fill in the implementation details in `Recognize()`.

**object_recognizer.h**
```cpp
#ifndef _PERCEPTION_OBJECT_RECOGNIZER_H_
#define _PERCEPTION_OBJECT_RECOGNIZER_H_

#include <string>
#include <vector>

#include "pcl/kdtree/kdtree_flann.h"

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace perception {
void LoadData(const std::string& data_dir,
              std::vector<perception_msgs::ObjectFeatures>* dataset);

class ObjectRecognizer {
 public:
  explicit ObjectRecognizer(
      const std::vector<perception_msgs::ObjectFeatures>& dataset);
  void Recognize(const Object& object, std::string* name, double* confidence);

 private:
  std::vector<perception_msgs::ObjectFeatures> dataset_;
};
}  // namespace perception

#endif  // _PERCEPTION_OBJECT_RECOGNIZER_H_
```

**object_recognizer.cpp**
```cpp
#include "perception/object_recognizer.h"

#include <limits.h>
#include <math.h>
#include <string>
#include <vector>

#include "boost/filesystem.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "perception/feature_extraction.h"
#include "perception_msgs/ObjectFeatures.h"
#include "ros/ros.h"

using boost::filesystem::directory_iterator;
using perception_msgs::ObjectFeatures;

namespace perception {
namespace {
double EuclideanDistance(const std::vector<double>& v1,
                         const std::vector<double>& v2) {
  // TODO: implement
}
}

void LoadData(const std::string& data_dir,
              std::vector<perception_msgs::ObjectFeatures>* dataset) {
  directory_iterator end;
  for (directory_iterator file_it(data_dir); file_it != end; ++file_it) {
    if (boost::filesystem::is_regular_file(file_it->path())) {
      rosbag::Bag bag;
      bag.open(file_it->path().string(), rosbag::bagmode::Read);
      std::vector<std::string> topics;
      topics.push_back("object_features");
      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
        ObjectFeatures::ConstPtr fp = it->instantiate<ObjectFeatures>();
        if (fp != NULL) {
          dataset->push_back(*fp);
        }
      }
    }
  }
}

ObjectRecognizer::ObjectRecognizer(const std::vector<ObjectFeatures>& dataset)
    : dataset_(dataset) {}

void ObjectRecognizer::Recognize(const Object& object, std::string* name,
                                 double* confidence) {
  // TODO: extract features from the object

  double min_distance = std::numeric_limits<double>::max();
  double second_min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < dataset_.size(); ++i) {
    // TODO: compare the features of the input object to the features of the current dataset object.
    if (distance < min_distance) {
      second_min_distance = min_distance;
      min_distance = distance;
      *name = dataset_[i].object_name;
    } else if (distance < second_min_distance) {
      second_min_distance = distance;
    }
  }

  // Confidence is based on the distance to the two nearest results.
  *confidence = 1 - min_distance / (min_distance + second_min_distance);
}
}  // namespace perception
```

Now update your CMakeLists.txt:
```cmake
add_library(perception_object_recognizer src/object_recognizer.cpp)
add_dependencies(perception_object_recognizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_object_recognizer ${catkin_LIBRARIES})
```

Make sure your code compiles at this point.

## Update the point cloud demo
Now we will update the point cloud demo to create a recognizer and pass it into the segmenter class.

**point_cloud_demo.cpp**
```cpp
#include <vector>

#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"

...

if (argc < 2) {
  ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
  ros::spinOnce();
}
std::string data_dir(argv[1]);

...

// Create the object recognizer.
std::vector<perception_msgs::ObjectFeatures> dataset;
perception::LoadData(data_dir, &dataset);
perception::ObjectRecognizer recognizer(dataset);

perception::Segmenter segmenter(table_pub, above_table_pub, marker_pub,
                                recognizer);
```

You will need to update `segmentation.cpp` and `segmentation.h` to take in the object recognizer:

**segmentation.h**
```cpp
#include "perception/object_recognizer.h"

...

  Segmenter(const ros::Publisher& surface_points_pub,
            const ros::Publisher& above_surface_pub,
            const ros::Publisher& marker_pub,
            const ObjectRecognizer& recognizer);

...

 private:
  ...
  ObjectRecognizer recognizer_;
```

In your segmenter, call the object recognizer for each object and show a text marker with the name and confidence scores:
**segmentation.cpp**
```cpp
#include <math.h>
#include <sstream>
#include "perception/object_recognizer.h"

Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
                     const ros::Publisher& above_surface_pub,
                     const ros::Publisher& marker_pub,
                     const ObjectRecognizer& recognizer)
    : surface_points_pub_(surface_points_pub),
      above_surface_pub_(above_surface_pub),
      marker_pub_(marker_pub),
      recognizer_(recognizer) {}

...
    // Publish a bounding box around the object
...
    // Recognize the object.
    // std::string name;
    // double confidence;
    // TODO: recognize the object with the recognizer_. /////////////////////////////
    confidence = round(1000 * confidence) / 1000;

    std::stringstream ss;
    ss << name << " (" << confidence << ")";

    // Publish the recognition result.
    visualization_msgs::Marker name_marker;
    name_marker.ns = "recognition";
    name_marker.id = i;
    name_marker.header.frame_id = "base_link";
    name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    name_marker.pose.position = object.pose.position;
    name_marker.pose.position.z += 0.1;
    name_marker.pose.orientation.w = 1;
    name_marker.scale.x = 0.025;
    name_marker.scale.y = 0.025;
    name_marker.scale.z = 0.025;
    name_marker.color.r = 0;
    name_marker.color.g = 0;
    name_marker.color.b = 1.0;
    name_marker.color.a = 1.0;
    name_marker.text = ss.str();
    marker_pub_.publish(name_marker);
```

Now, update your CMakeLists.txt to reflect the fact that these facts:
- The point cloud demo now depends on the object recognizer library
- The segmentation code now depends on the feature extraction and object recognizer libraries

Finally, pass in the data_dir as an argument to your point cloud demo.
```xml
<launch>
  <arg name="data_dir" />
  <param name="ec_cluster_tolerance" value="0.03" />
  <param name="ec_min_cluster_size" value="700" />
  <param name="ec_max_cluster_size" value="7500" />
  <param name="distance_above_plane" value="0.01" />
  <param name="crop_min_x" value="-0.5" />
  <param name="crop_max_z" value="1.5" />
  <param name="crop_max_x" value="1.05" />
  <param name="crop_min_z" value="0.7" />
  <node pkg="perception" type="point_cloud_demo" name="point_cloud_demo" args="$(arg data_dir)" output="screen">
    <remap from="cloud_in" to="mock_point_cloud" />
  </node>
</launch>
```

## Run the demo
```
roslaunch perception point_cloud_demo data_dir:=~/data/objects/labels
```

Here is what our output looks like.
As you can see, it can recognize the right object classes, but it can't distinguish between differently-colored objects.
Note that the confidence score is lower for objects that are ambiguously shaped, but higher for uniquely shaped objects.
![image](https://cloud.githubusercontent.com/assets/1175286/26271886/da752158-3cc0-11e7-9f42-0e1c0446d901.png)

# Step 5: Extension to color
Now that we have tried recognizing objects based on size, we can try recognizing objects based on color.
Luckily, we have already written all the infrastructure we need.
Now, we just need to write a different feature extractor.

First, we will add an `ExtractColorFeatures` function.
Edit `feature_extraction.h` and `feature_extraction.cpp`:

**feature_extraction.h**
```cpp
void ExtractColorFeatures(const Object& object,
                          perception_msgs::ObjectFeatures* features);
```

**feature_extraction.cpp**
```cpp
void ExtractColorFeatures(const Object& object,
                          perception_msgs::ObjectFeatures* features) {
  std::vector<double> color_features;
  color_features.resize(125);
  for (size_t i = 0; i < object.cloud->size(); ++i) {
    const pcl::PointXYZRGB& pt = object.cloud->at(i);
    // Reduce the color space to just 5 values (255 / 51) per channel.
    int r = std::min(pt.r / 51, 4);
    int g = std::min(pt.g / 51, 4);
    int b = std::min(pt.b / 51, 4);
    int index = r * 25 + g * 5 + b;
    color_features[index] += 1;
  }

  // Normalize to get a distribution.
  for (size_t i = 0; i < color_features.size(); ++i) {
    color_features[i] /= object.cloud->size();
  }

  features->values.insert(features->values.end(), color_features.begin(),
                          color_features.end());
}
```

Now replace `ExtractSizeFeatures` with `ExtractColorFeatures` in `src/extract_features_main.cpp` and `src/segmentation.cpp`.

Test out your feature extraction on the blue cup:
```
rosrun perception extract_features blue_cup.bag blue_cup
rostopic echo object_features
rosbag play blue_cup_label.bag
```

Here is what we get for the blue cup:
```
object_name: blue_cup
names: []
values: [0.0, 0.0004371584699453552, 0.006557377049180328, 0.01377049180327869, 0.0004371584699453552, 0.0, 0.0, 0.0, 0.002622950819672131, 0.09595628415300546, 0.0, 0.0, 0.0, 0.0, 0.0004371584699453552, 0.0, 0.0, 0.0, 0.0, 0.0002185792349726776, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0008743169398907104, 0.0, 0.0, 0.0002185792349726776, 0.0002185792349726776, 0.002622950819672131, 0.048743169398907105, 0.3770491803278688, 0.0, 0.0, 0.0, 0.0006557377049180328, 0.005027322404371585, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0002185792349726776, 0.0008743169398907104, 0.0013114754098360656, 0.006557377049180328, 0.0015300546448087432, 0.0, 0.0024043715846994535, 0.0004371584699453552, 0.0030601092896174863, 0.009617486338797814, 0.0, 0.0, 0.0, 0.0, 0.0002185792349726776, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0002185792349726776, 0.0, 0.0, 0.0008743169398907104, 0.0006557377049180328, 0.005027322404371585, 0.0028415300546448087, 0.0, 0.0, 0.0017486338797814208, 0.0015300546448087432, 0.004371584699453552, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0004371584699453552, 0.0, 0.0, 0.0, 0.0, 0.0006557377049180328, 0.008961748633879782, 0.004153005464480874, 0.004808743169398907, 0.0, 0.0, 0.0, 0.003715846994535519, 0.3779234972677596]
```

You can now extract features for all of the objects:
```
rosrun perception extract_features expo.bag expo
rosrun perception extract_features galaxy_nexus.bag galaxy_nexus
rosrun perception extract_features orange_dog_toy.bag orange_dog_toy
rosrun perception extract_features red_cup.bag red_cup
rosrun perception extract_features turtle.bag turtle
rosrun perception extract_features yellow_dog_toy.bag yellow_dog_toy
```

Move all of your labeled data into a separate folder
```
cd ~/data/objects
mkdir color_labels
mv *_label.bag color_labels
```

Now run your recognizer on this new data directory:
```
roslaunch perception point_cloud_demo.launch data_dir:=/home/jstn/data/objects/color_labels/
```

![image](https://cloud.githubusercontent.com/assets/1175286/26272314/b8912bc8-3cc9-11e7-9849-ae9784ee85a5.png)

Better, but it still confuses the blue cup with the expo marker.
It's true that both the cup and the expo marker are mostly blue and have a light color on them.

Let's make a final refinement and use both size and color features.

Create a third feature extraction function called `ExtractFeatures`:
```
void ExtractFeatures(const Object& object,
                     perception_msgs::ObjectFeatures* features) {
  ExtractSizeFeatures(object, features);
  ExtractColorFeatures(object, features);
}
```

"Size" and "color probability" exist in totally different domains.
There is no guarantee that their relative magnitudes make any sense.
As a result, we should add an adjustable weight to the size features, so that we can control the relative importance of the size features compared to the color features.
In this lab, we will manually tune this weight.
Machine learning is the process of automatically tuning the weight based on a dataset.

```cpp
void ExtractSizeFeatures(const Object& object,
                         perception_msgs::ObjectFeatures* features) {
  double weight;
  ros::param::param("size_weight", weight, 2.0);
  // "x" dimension is always the smallest of x and y to account for rotations.
  // z always points up.
  features->names.push_back("box_dim_x");
  features->values.push_back(
      weight * std::min(object.dimensions.x, object.dimensions.y));
  features->names.push_back("box_dim_y");
  features->values.push_back(
      weight * std::max(object.dimensions.x, object.dimensions.y));
  features->names.push_back("box_dim_z");
  features->values.push_back(weight * object.dimensions.z);
}
```

It will be tedious to regenerate the labels every time you tune the weight.
You can write a small script to automate the process:

```
cd ~/data/objects
touch extract_features.sh
chmod +x extract_features.sh
vim extract_features.sh
```

**extract_features.sh**
```bash
. /opt/ros/indigo/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf combined_labels
mkdir combined_labels
rosrun perception extract_features blue_cup.bag blue_cup
rosrun perception extract_features expo.bag expo
rosrun perception extract_features galaxy_nexus.bag galaxy_nexus
rosrun perception extract_features orange_dog_toy.bag orange_dog_toy
rosrun perception extract_features red_cup.bag red_cup
rosrun perception extract_features turtle.bag turtle
rosrun perception extract_features yellow_dog_toy.bag yellow_dog_toy
mv *_label.bag combined_labels
```

Now you can run the following steps:
```
cd ~/data/objects
rosparam set size_weight 2
./extract_features.sh
roslaunch perception point_cloud_demo.launch data_dir:=/home/jstn/data/objects/combined_labels/
```

You will notice that with a low weight for the size features, the system is still confusing the blue cup for the expo marker.
This suggests that you need to further emphasize the importance of the size.

After tweaking the `size_weight` param, this is what we get:
![image](https://cloud.githubusercontent.com/assets/1175286/26272456/47e1bdda-3ccd-11e7-9313-4ab188b2f1b1.png)