So far, we've seen how to control the robot's joints.
But what about reading them?
How, for example, will we know the current height of the torso or whether the gripper is opened or closed?
Another common use case is moving the arm in joint space.
We could just guess and check values for the arm, but it would be nice to just move the arm to the desired pose and then read off the joint states.

# The /joint_states topic
Luckily for us, the robot continuously publishes the current joint angles to the `/joint_states` topic.
However, each message might only contain a subset of the joints.
This works because multiple nodes can publish the state of the subset of joints they are responsible for.
This is how `/joint_states` works on the real robot.
However, in simulation, all of the joint states are published by Gazebo.

Because no single message on the `/joint_states` topic will necessarily tell us about all the joints of the robot, we must listen to multiple messages over time and accumulate the full state of the robot.
In practice, the joint states are published very quickly, so we will not have to wait long.

* What is the type of the `/joint_states` topic?
* Use `rosmsg show -r TYPE` to show the message definition, including comments

# Create the joint_state_reader package
We will create a library that accumulates messages on `/joint_states` and can give the latest joint value for any joint.
Because this functionality is generic to many kinds of robots (not just the Fetch), we will create a separate package for it:

```
cd ~/catkin_ws/src/cse481c
catkin create pkg joint_state_reader --catkin-deps roscpp rospy sensor_msgs
```

# Create the starter files

```
mkdir joint_state_reader/src/joint_state_reader
```

**`joint_state_reader/src/joint_state_reader/__init__.py`**
```py
from .reader import JointStateReader
```

**`joint_state_reader/src/joint_state_reader/reader.py`**
```py
#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy                                                                                           
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                
    def __init__(self):                                                                                
        pass                                                                                           
                                                                                                       
    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """                                                                                            
        rospy.logerr('Not implemented.')                                                               
        return 0                                                                                       
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """                                                                     
        rospy.logerr('Not implemented.')                                        
        return [0 for x in names]
```

**`joint_state_reader/setup.py`**
```py
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['joint_state_reader'],
    package_dir={'': 'src'})

setup(**setup_args)
```

Finally, edit **`CMakeLists.txt`** and uncomment `catkin_python_setup()`.

# Build your catkin workspace
Because we have created a new package, we need to make catkin aware of it by building the workspace:
```
catkin build
```

You will also need to re-source your .bashrc in each terminal window:
```
source ~/.bashrc
```

# Create the demo program
We will need to configure the `applications` package to depend on `joint_state_reader`.
Edit `package.xml` and add the following:
```xml
<depend>joint_state_reader</depend>
```

Now create the following demo file:
**`applications/scripts/joint_reader_demo.py`**
```py
#! /usr/bin/env python                                                                                 
                                                                                                       
import fetch_api                                                                                       
from joint_state_reader import JointStateReader
import rospy                  
                              
                       
def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass                                                                                           
                                                                                                       
                                                                                                       
def main():                                                                                            
    rospy.init_node('joint_reader_demo')                                                               
    wait_for_time()                                                                                    
    argv = rospy.myargv()                                                                              
    reader = JointStateReader()
    rospy.sleep(0.5)
    names = fetch_api.ArmJoints.names()
    arm_vals = reader.get_joints(names)
    for k, v in zip(names, arm_vals):
        print '{}\t{}'.format(k, v)
                      
                      
if __name__ == '__main__':
    main()
```

Now, running the demo should produce the following:
```
rosrun applications joint_reader_demo.py

/joint_reader_demo get_joints:22: Not implemented.
shoulder_pan_joint      0
shoulder_lift_joint     0
upperarm_roll_joint     0
elbow_flex_joint        0
forearm_roll_joint      0
wrist_flex_joint        0
wrist_roll_joint        0
```

# Implement the joint reader
Hopefully, it should be clear what to do.
If not, here are some hints:
- You need to subscribe to `/joint_states`
- For each msg that you get, you need to add or update the latest joint values to a data structure
- When you call `get_joint` or `get_joints`, you read the values out from that data structure and return them.

With the arm in the "tucked" position (restart your simulator if your arm has moved), you should get output similar to this (but not exactly):
```
shoulder_pan_joint      1.32000351327
shoulder_lift_joint     1.39998150631
upperarm_roll_joint     -0.199847344952
elbow_flex_joint        1.71997036482
forearm_roll_joint      7.55260950402e-06
wrist_flex_joint        1.66001107158
wrist_roll_joint        1.28453731207e-06
```

Try reading other joints, such as `l_gripper_finger_joint` or `torso_lift_joint`.