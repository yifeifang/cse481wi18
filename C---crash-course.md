ROS uses C++ and Python as its main languages.
C++ tends to be used for more computationally expensive and low-level tasks, while Python is useful for high-level scripting and rapid development.
This document serves as a refresher in C++ and discusses features commonly seen in ROS.

# Main files
Main files (those with a `main` function) go in the `src` folder.
For short utility tools, I like to add `_main` to the file name, e.g,. `hello_main.cpp`, whereas for long-running ROS nodes, I like to add `_node`.
Here is an example of a node for a map annotator server.

**src/map_annotator_node.cpp**
```cpp
#include "map_annotator/server.h"
#include "visualization_msgs/Marker.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "hello");
  ros::NodeHandle nh;

  ros::Publisher viz_pub = nh.advertise<visualization_msgs::Marker>("visualization_markers", 5);
  map_annotator::Server server(viz_pub);

  // This is how to make a method of a class a callback.
  ros::Subscriber sub = nh.subscribe("user_actions", 10, map_annotator::Server::HandleUserAction, &server);

  ros::spin();
  return 0;
}
```

# Library files
All other code (without a main function) can be classified as library code.
Typically you split your library code into header files (.h) and source files (.cpp).
Header files declare functions and classes, but does not include any code to implement anything.
The implementations are done in the source files.
Here is an example of a class that acts as a map annotator server:

**include/map_annotator/server.h**
```cpp
// Normally, messages are created in separate packages that exist only to hold messages.
// For example, we should have created UserAction.msg in a package named map_annotator_msgs.
// This is so that you don't have to depend on a ton of code to use a topic/service/action.
// In our previous labs, we just created a message in the map_annotator package.
// Notice that in C++, creating a message in this package autogenerates a class called UserAction.
// This prevents us from creating our own class called UserAction in the future.
#include "map_annotator/UserAction.h"
#include "ros/ros.h"

namespace map_annotator {
class Server {
 public:
  // Constructor.
  Server(const ros::Publisher& pub);

  // Call this method to have the server handle a user action.
  // This can also be used as the callback to a subscriber.
  void HandleUserAction(const UserAction& user_action);
 private:
  ros::Publisher pub_; // Private members have a trailing underscore by convention.
};
}
```

**src/server.cpp**
```cpp
#include "map_annotator/server.h"

#include "map_annotator/UserAction.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

namespace map_annotator {
// Constructor, copies the passed-in publisher.
Server::Server(const ros::Publisher& pub): pub_(pub) {}

void Server::HandleUserAction(const UserAction& user_action) {
  if (...) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    pub_.publish(marker);
  }
}
}
```

# Data structures
## std::vector
[`std::vector`](http://www.cplusplus.com/reference/vector/vector/) is the equivalent of a list in Python:

```cpp
#include <vector>

std::vector<geometry_msgs::PoseStamped> locations;
locations.push_back(pose_stamped);
vec.size();
for (const PoseStamped& ps : locations) {
  // Do something with ps.
}
```

## std::map and std::unordered_map
[`std::map`](http://www.cplusplus.com/reference/map/map/) and [`std::unordered_map`](http://www.cplusplus.com/reference/unordered_map/unordered_map/) are the equivalents of dictionaries in Python.
A map is implemented with a balanced search tree and an unordered map is implemented with a hash table.

```cpp
#include <map>

std::map<std::string, geometry_msgs::PoseStamped> locations;
locations["table"] = pose_stamped;
if (locations.find("door") == locations.end()) {
}
for (const auto& kv : locations) {
  const std::string& name = kv.first;
  const geometry_msgs::PoseStamped& ps = kv.second;
}
```

# Namespaces
Everything is defined at the top level unless it's put in a namespace.
Generally, you should put your code in a namespace named after your project.
Otherwise, your names may collide with other names defined at the top level.

```cpp
namespace map_annotator {
class Server {
  ...
};

void DoSomething();
}
```

To refer to something, you have to prepend the namespace it's in, unless it's in the same namespace that you are currently in:
```cpp
map_annotator::Server server;
map_annotator::DoSomething();

namespace map_annotator {
void DoSomething2() {
  Server server;
  DoSomething();
}
}
```

## `using` keyword

You can use the `using` keyword at the top of your file to reduce the amount of typing you have to do.
```cpp
#include <string>
#include <vector>

// Now instead of typing std::string everywhere you can just type string.
using std::string;

int main(int argc, char** argv) {
  std::vector<string> strs;
}
```

You can also use the `using namespace` keyword to import all names from a namespace.
However, you should not do this in header files.
This is because those names are not just available in that header file, but also any files that include that header file, and any files that include any files that include that header file, etc.

**main.cpp**
```cpp
#include <string>
#include <vector>

using namespace std;

int main(int argc, char** argv) {
  vector<string> strs;
}
```

# References and const references
You have seen so far many functions that look like this:
```cpp
void DoSomething(const geometry_msgs::PoseStamped& ps) {
}
```

In C++, arguments are passed to a function can either be by value (no annotation), a pointer (add a `*` to the type), or by reference (add a `&` to the type).
- Passing by value makes a copy, which is less efficient for large objects.
- Passing a pointer avoids making a copy and allows the function to modify the argument.
- Passing by reference is the same as passing by pointer, but it looks like passing by value.
- By adding `const` to a reference, we get the best of both worlds for arguments that represent read-only input. Large objects can be passed in without making a copy (like passing a pointer), but the function cannot modify the argument (like passing by value).

```cpp
void MaybeIncrement(int a, int* bp, int& c, const int& d) {
  a += 1; // Does nothing.
  *bp += 1; // Increments where bp points to.
  c += 1; // Increments 1.
  d += 1; // Compiler error.
}

void Test() {
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  MaybeIncrement(a, &b, c, d);
  // a = 0
  // b = 1
  // c = 1
}
```

This arises for all assignments:
```cpp
Pose pose = pose_stamped.pose; // Makes a copy.
// Doesn't make a copy, but modifying pose won't modify pose_stamped.pose, because the compiler won't let you.
const Pose& pose = pose_stamped.pose;
```

A common function signature is:
```cpp
void DoSomething(const PoseStamped& input1, int input2, string* output1, string* output2);

string output1;
string output2;
DoSomething(input1, input2, &output1, &output2);
```
Inputs go first in the argument list and are passed in as const references if they do not need to be modified.
Outputs go last and are passed as pointers.
This way, you can look at a call site like `DoSomething(input1, input2, &output1, &output2)` and immediately see that any args with an `&` in front of them might be changed by that function.

# Shared pointers
This is bad:
```cpp
void Evil(Pose* pose) {
  delete pose;
}

void main() {
  Pose* pose = new Pose();
  Evil(pose);
  std::cout << pose.x << std::endl; // Segfault.
}
```

If you use pointers, you must carefully keep track of who is responsible for deleting them (this is called *ownership*).

In ROS, you are likely to find many instances of `boost::shared_ptr`, which is an object that wraps around a pointer.
A `shared_ptr` keeps track of who is using that pointer, and once that last `shared_ptr` goes out of scope, the memory is freed automatically.

Here is the basic usage:
```cpp
// Okay to pass shared_ptr by value, but you can also make it into a const reference.
void Process(boost::shared_ptr<sensor_msgs::PointCloud2> cloud_ptr) {
  cloud_ptr->header.frame_id = "camera_frame";
  (*cloud_ptr).header.frame_id = "camera_frame";
}

void DoSomething() {
  // Allocate the pointer with the new keyword in the constructor.
  boost::shared_ptr<sensor_msgs::PointCloud2> cloud_ptr(new sensor_msgs::PointCloud2());
  // Memory is automatically deleted after cloud_ptr goes out of scope.
}
```

In ROS, it is common to find types with `Ptr` or `ConstPtr` at the end.
These are typedefs for `boost:shared_ptr`s.
For example, `sensor_msgs::PointCloud2Ptr` is a `boost::shared_ptr<sensor_msgs::PointCloud2>`.