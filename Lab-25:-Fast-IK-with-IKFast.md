In Lab 19, we bragged about how the Fetch arm is one of the kinds of arms that have a closed-form IK solution.
But, as you might have noticed from previous labs, running IK is extremely slow, taking at least several seconds to find a solution.
Why is this happening?

MoveIt is designed to work for all kinds of robot arms.
Because not all arms have closed-form IK solutions, MoveIt is configured by default to use an iterative IK solver that works for for all arms.
This is why solving IK is slow.

In this lab, we will replace the IK solver with a faster one that computes the closed-form solution that the Fetch supports.

# IKFast
The faster IK solver is called `IKFast`.
After configuring `IKFast` for your robot, we then generate C++ code to compute IK solutions.
MoveIt uses a plugin system to be able to swap out different algorithms for IK, planning, etc.
Luckily, Fetch Robotics has already generated the IKFast plugin for us, so we just need to install it on our computers.

# Get the plugin

Follow these instructions to download the pre-generated IKFast plugin and build it.
```
mkdir ~/local
cd ~/local
git clone git@github.com:fetchrobotics/fetch_ros.git
cp -r fetch_ros/fetch_ikfast_plugin ~/catkin_ws/src/cse481c/
cd ~/catkin_ws/src/cse481c
catkin build fetch_ikfast_plugin
```

# Configure MoveIt
Next, we need to configure MoveIt to use this new plugin.
To do this, we will create modified versions of the MoveIt launch files.

```
source ~/.bashrc
mkdir ~/catkin_ws/src/cse481c/fetch_api/launch
mkdir ~/catkin_ws/src/cse481c/fetch_api/config
cd ~/catkin_ws/src/cse481c/fetch_api
roscp fetch_moveit_config move_group.launch launch
roscp fetch_moveit_config planning_context.launch launch
roscp fetch_moveit_config kinematics.yaml config
```

Edit `fetch_api/config/kinematics.yaml` to look like this:
```yaml
arm:
  kinematics_solver: fetch_arm_kinematics/IKFastKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
arm_with_torso:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
```

Note that this new, faster IK will only work with the `arm` MoveGroup, not `arm_with_torso`.

Edit `fetch_api/launch/move_group.launch` and replace this:
```xml
<include file="$(find fetch_moveit_config)/launch/planning_context.launch" />
```
with:
```xml
<include file="$(find fetch_api)/launch/planning_context.launch" />
```

Finally, edit `fetch_api/launch/planning_context.launch` and replace this:
```xml
<rosparam command="load" file="$(find fetch_moveit_config)/config/kinematics.yaml"/>
```
with:
```xml
<rosparam command="load" file="$(find fetch_api)/config/kinematics.yaml"/>
```

# Run MoveIt
Shut down MoveIt if it is already running.
Then, reload your .bashrc:
```
source ~/.bashrc
```

**From now on, this is how you launch MoveIt:**
```
roslaunch fetch_api move_group.launch
```
Make sure that when you launch MoveIt, there are no red error messages.

Now, try computing IK:
```
rosrun applications check_cart_pose.py ik 1 0 1
```

It should run much faster.
Note that there is still some overhead associated with starting a node and calling a service.