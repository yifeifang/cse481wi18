In this lab, you will create a new hybrid Python/C++ package named `perception`.
We will start the perception package by writing a basic C++ program to save point clouds from the robot to a file.

# Create the package
This is the same as creating any other package:
```
cd ~/catkin_ws/src/cse481c
catkin create pkg perception --catkin-deps roscpp rospy
```

# Write the code
We will write a simple program that waits for a point cloud, serializes it, and saves it to a file.
Create a file in `perception/src/save_cloud_main.cpp`:
```cpp
#include <iostream>
#include <string>

#include "ros/ros.h"

void print_usage() { 
  std::cout << "Saves a point cloud on head_camera/depth_registered/points to "
               "NAME.bag in the current directory."
            << std::endl;
  std::cout << "Usage: rosrun perception save_cloud NAME" << std::endl;
}

int main(int argc, char** argv) { 
  ros::init(argc, argv, "save_cloud_main");
  if (argc < 2) { 
    print_usage();
    return 1;
  } 
  std::string name(argv[1]);
  std::cout << "Hello, " << name << std::endl;

  return 0;
}
```

# Set up the build
Let's make sure that our skeleton code above compiles.
When building C++ code, you must configure catkin by editing `package.xml` and `CMakeLists.txt`.
`package.xml` is a catkin-specific file that lists what ROS packages your package depends on.
`CMakeLists.txt` is a [CMake](https://cmake.org/) config file with special catkin-specific code in it.

In `package.xml`, you will add a `<depend>` tag for each ROS package that your code depends on.
All ROS C++ code must depend on the `roscpp` package.

`CMakeLists.txt` looks daunting at first.
We will discuss everything that goes in it.
Ignoring comments, try to make your CMakeLists.txt look like this:
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(perception_save_cloud_main src/save_cloud_main.cpp)
set_target_properties(perception_save_cloud_main PROPERTIES OUTPUT_NAME save_cloud PREFIX "")
add_dependencies(perception_save_cloud_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_save_cloud_main
   ${catkin_LIBRARIES}
)
```

Compile your code:
```
catkin build
```

Because we created a new package, we should see the message:
```
Note: Workspace packages have changed, please re-source setup files to use them.
```

This means that you need to re-source your .bashrc in each terminal window for ROS to be aware of the new package:
```
source ~/.bashrc
```

You can now run the skeleton code:
```
> rosrun perception save_cloud Justin
Hello, Justin
```

# Understanding CMakeLists.txt
Hopefully, you already have some experience building C++ code.
If so, then you know that to build a program, you run `g++` like this:
```
# NOT a real command. Illustration purpose only.
g++ save_cloud_main.cpp -I /opt/ros/indigo/include -lroscpp -o save_cloud
```

This says:
- Compile code in the files: save_cloud_main.cpp
- Search for header files (When using `#include`) in `/opt/ros/include`
- Link with the `roscpp` library
- Name the output executable `save_cloud`

You can remember this with the acronym COIL: **C**ode, **O**utput, **I**ncludes, **L**ibraries.
All of the elements of COIL are specified in `CMakeLists.txt` as well, just in a different way:
```cmake
include_directories( # INCLUDES
  ${catkin_INCLUDE_DIRS}
)

add_executable(perception_save_cloud_main src/save_cloud_main.cpp) # CODE TO COMPILE
set_target_properties(perception_save_cloud_main PROPERTIES OUTPUT_NAME save_cloud PREFIX "") # OUTPUT NAME
...
target_link_libraries(perception_save_cloud_main # LIBRARIES TO LINK WITH
   ${catkin_LIBRARIES}
)
```

Catkin saves you from having to tediously write the locations of the include directory and the libraries for each and every ROS package you use.
Instead, you list all the packages you depend on at the top:
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
```

All of the include directories and libraries from `roscpp` and `rospy` get loaded into the variables `${catkin_INCLUDE_DIRS}` and `${catkin_LIBRARIES}`.
As we depend on more ROS packages, we just add them to the `find_package` section.
Note: you can depend on other packages in your catkin workspace.

## Writing a library
If you write an awesome algorithm and want to use it in other code, you can export it as a library.
Assuming you wrote your algorithm in `src/awesome_algorithm.cpp` and `include/perception/awesome_algorithm.h`, here's how that looks:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES perception_awesome_algorithm
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(perception_awesome_algorithm src/awesome_algorithm.cpp)
add_dependencies(perception_awesome_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_awesome_algorithm
   ${catkin_LIBRARIES}
)

add_executable(perception_save_cloud_main src/save_cloud_main.cpp)
set_target_properties(perception_save_cloud_main PROPERTIES OUTPUT_NAME save_cloud PREFIX "")
add_dependencies(perception_save_cloud_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_save_cloud_main
   perception_awesome_algorithm
   ${catkin_LIBRARIES}
)
```

Here, we create a library named `perception_awesome_algorithm` with `add_library`.
We link it to `roscpp` via `target_link_libraries(perception_awesome_algorithm ${catkin_LIBRARIES})`.
We also specify that our package contains header files in the `include` folder, using `include_directories` and `catkin(INCLUDE_DIRS include)`.
The former is needed to successfully compile our code, while the latter is necessary to export our header files so that other packages can find `awesome_algorithm.h`

In this example, we are assuming that the `save_cloud` executable wants to use the awesome algorithm.
In that case, it must link against the `perception_awesome_algorithm` library by listing it in `target_link_libraries`.

# Reading a point cloud
To read a point cloud, we will use `ros::topic::wait_for_message`:
```cpp
#include "sensor_msgs/PointCloud2.h"
...
sensor_msgs::PointCloud2ConstPtr cloud =
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        "head_camera/depth_registered/points");
```

We will also transform the cloud into `base_link` using the `pcl_ros` package's [`transformPointCloud`](http://docs.ros.org/indigo/api/pcl_ros/html/namespacepcl__ros.html#ae730596163b8fa79f374faa0eae1c53f).
It is important to transform this into a fixed frame now because left untransformed, the point cloud depends on how the head is tilted.
This code looks up the latest transform between the cloud frame and the base frame and transforms the point cloud into the base frame.

```cpp
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
...
tf::TransformListener tf_listener;                                                    
tf_listener.waitForTransform("base_link", cloud->header.frame_id,                     
                             ros::Time(0), ros::Duration(5.0));                       
tf::StampedTransform transform;                                                       
try {                                                                                 
  tf_listener.lookupTransform("base_link", cloud->header.frame_id,                    
                              ros::Time(0), transform);                               
} catch (tf::LookupException& e) {                                                    
  std::cerr << e.what() << std::endl;                                                 
  return 1;                                                                           
} catch (tf::ExtrapolationException& e) {                                             
  std::cerr << e.what() << std::endl;                                                 
  return 1;                                                                           
}                                                                                     
                                                                                        
sensor_msgs::PointCloud2 cloud_out;                                                   
pcl_ros::transformPointCloud("base_link", transform, *cloud, cloud_out);
```

Finally, we will write the cloud to a [bag file](http://wiki.ros.org/rosbag/Code%20API):
```cpp
#include "rosbag/bag.h"
...
std::string filename(name + ".bag");
rosbag::Bag bag;
bag.open(filename, rosbag::bagmode::Write);
bag.write("head_camera/depth_registered/points", ros::Time::now(), cloud_out);
bag.close();

return 0;
```

# Building the cloud saver
Before you build, remember that you started depending on four new packages:
- pcl_ros
- sensor_msgs
- tf
- rosbag

Add a `<build_depend>` and a `<run_depend>` for each of those four packages to your `package.xml`.

Next, add those three packages to `find_package` in `CMakeLists.txt`:
```cmake
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  rosbag       
  roscpp    
  rospy
  sensor_msgs
  tf
)
```

Now, your code should compile:
```
catkin build
```

With the simulator open, you should be able to record a point cloud:
```
rosrun perception save_cloud cloud
rosbag info cloud.bag
```

# Save a point cloud from the real robot
With six teams and one robot, finding ways to work in simulation as much as possible will be important.
The next lab will introduce AR tags.
The classroom has been set up with a few AR tags in the scene.

Save a point cloud of the scene to use for later.

First, use RViz and make sure that the point cloud has all three tags in it:
```
setrobot astro
rosrun rviz rviz
```

Then, save a point cloud:
```
setrobot astro
rosrun perception save_cloud tags
```