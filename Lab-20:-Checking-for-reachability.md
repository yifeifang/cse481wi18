In the previous lab, we saw how to move the robot's arm using motion planning.
However, what if we just want to just check whether a gripper pose is feasible, without actually running it on the robot?
In this lab, we will explore two ways of getting this information: motion planning and inverse kinematics.

To use motion planning, we will use nearly identical code as in the previous lab, but we will pass in some arguments that we have not yet explored.
The difference between motion planning and inverse kinematics is that motion planning will tell you whether a pose is reachable from where the arm is right now, including obstacles that must be avoided.
Inverse kinematics, however, tells you whether a pose is theoretically feasible, regardless of where the arm currently is and what obstacles there are.

For example, suppose you have a sequence of five poses you want to execute.
You can use inverse kinematics on each of those five poses to tell you if each of those is theoretically reachable.
You can then use the motion planning approach to tell you whether the *first* pose is reachable from the current arm position.
Once you are at the first pose, then you can run the motion planner again to tell you if the *second* pose is reachable, etc.

# Check the plan
Look at the documentation for [MoveItGoalBuilder](https://github.com/cse481wi18/cse481wi18/blob/indigo-devel/fetch_api/src/fetch_api/moveit_goal_builder.py).
You can see that it supports many possible arguments.
Although we will not use all of these arguments, we will add support for many of these arguments now.

The argument of relevance right now is `plan_only`, which is set to `False` by default.
Setting this to `True` means that the motion planner will come up with a plan, but not execute it on the robot.

Update the signature of `move_to_pose` in your `Arm` class to be:
```py
def move_to_pose(self,
                 pose_stamped,
                 allowed_planning_time=10.0,
                 execution_timeout=15.0,
                 group_name='arm',
                 num_planning_attempts=1,
                 plan_only=False,
                 replan=False,
                 replan_attempts=5,
                 tolerance=0.01):
    """Moves the end-effector to a pose, using motion planning.

    Args:
        pose: geometry_msgs/PoseStamped. The goal pose for the gripper.
        allowed_planning_time: float. The maximum duration to wait for a
            planning result, in seconds.
        execution_timeout: float. The maximum duration to wait for
            an arm motion to execute (or for planning to fail completely),
            in seconds.
        group_name: string. Either 'arm' or 'arm_with_torso'.
        num_planning_attempts: int. The number of times to compute the same
            plan. The shortest path is ultimately used. For random
            planners, this can help get shorter, less weird paths.
        plan_only: bool. If True, then this method does not execute the
            plan on the robot. Useful for determining whether this is
            likely to succeed.
        replan: bool. If True, then if an execution fails (while the arm is
            moving), then come up with a new plan and execute it.
        replan_attempts: int. How many times to replan if the execution
            fails.
        tolerance: float. The goal tolerance, in meters.

    Returns:
        string describing the error if an error occurred, else None.
    """
```

Now, pass all of these extra arguments to the `MoveItGoalBuilder`:
```py
    goal_builder = MoveItGoalBuilder()
    goal_builder.set_pose_goal(pose_stamped)
    goal_builder.allowed_planning_time = allowed_planning_time
    goal_builder.num_planning_attempts = num_planning_attempts
    goal_builder.plan_only = plan_only
    goal_builder.replan = replan
    goal_builder.replan_attempts = replan_attempts
    goal_builder.tolerance = tolerance
    goal = goal_builder.build()
    # Use execution_timeout for wait_for_result()
    self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))
```

If you just want to check if a pose is possible, you can call `move_to_pose` with `plan_only=True`.
You might want to add a convenience method to do this instead:
```py
def check_pose(self, 
               pose_stamped,
               allowed_planning_time=10.0,
               group_name='arm',
               tolerance=0.01):
    return self.move_to_pose(
        pose_stamped,
        allowed_planning_time=allowed_planning_time,
        group_name=group_name,
        tolerance=tolerance,
        plan_only=True)
```

Next, use this demo program, `check_cart_pose.py`  to test whether it works.
This tool will tell you whether the gripper can be moved to a certain pose, in the base frame, with the gripper pointing straight ahead (the identity orientation).
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


def print_usage():
    print 'Usage: rosrun applications check_cart_pose.py plan X Y Z'
    print '       rosrun applications check_cart_pose.py ik X Y Z'


def main():
    rospy.init_node('check_cart_pose')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 5:
        print_usage()
        return
    command, x, y, z = argv[1], float(argv[2]), float(argv[3]), float(argv[4])
   
    arm = fetch_api.Arm()
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.w = 1
           
    if command == 'plan':
        error = arm.check_pose(ps, allowed_planning_time=1.0)
        if error is None:
            rospy.loginfo('Found plan!')
        else:       
            rospy.loginfo('No plan found.')
        arm.cancel_all_goals()
    elif command == 'ik':
        if arm.compute_ik(ps):
            rospy.loginfo('Found IK!')
        else:
            rospy.loginfo('No IK found.')
    else:           
        print_usage()
                    
                    
if __name__ == '__main__':
    main()
```

You will need to run `move_group.launch` if it is not still open from the previous lab.
This is true whenever you want to do anything with MoveIt.
You should see:
```
rosrun applications check_cart_pose.py plan 0.5 0 1
/check_cart_pose main:41: Found plan!
rosrun applications check_cart_pose.py plan 1 0 1
/check_cart_pose main:41: No plan found.
```

# Inverse kinematics
You can compute the inverse kinematics given an end-effector pose by calling the `compute_ik` service provided by MoveIt.
If you look at the definition of the service, it can be quite intimidating.
However, only a few fields really need to be filled out; the rest is for "seeding" the IK with an approximate solution to get the result faster.

Add a method to `Arm` called `compute_ik`:

```py
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# In __init__ method
self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
```

We are feeling generous and will just give you the `compute_ik` method :).
Be sure to take a look and make sure you understand what's going on.

```py
def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
    """Computes inverse kinematics for the given pose.

    Note: if you are interested in returning the IK solutions, we have
        shown how to access them.

    Args:
        pose_stamped: geometry_msgs/PoseStamped.
        timeout: rospy.Duration. How long to wait before giving up on the
            IK solution.

    Returns: True if the inverse kinematics were found, False otherwise.
    """
    request = GetPositionIKRequest()
    request.ik_request.pose_stamped = pose_stamped
    request.ik_request.group_name = 'arm'
    request.ik_request.timeout = timeout
    response = self._compute_ik(request)
    error_str = moveit_error_string(response.error_code.val)
    success = error_str == 'SUCCESS'
    if not success:
        return False
    joint_state = response.solution.joint_state
    for name, position in zip(joint_state.name, joint_state.position):
        if name in ArmJoints.names():
            rospy.loginfo('{}: {}'.format(name, position))
    return True
```

Test your demo again:
```
rosrun applications check_cart_pose.py ik 0.5 0 1
/check_cart_pose compute_ik:206: shoulder_pan_joint: -0.9173148899
/check_cart_pose compute_ik:206: shoulder_lift_joint: 1.49589384052
/check_cart_pose compute_ik:206: upperarm_roll_joint: 1.83205661666
/check_cart_pose compute_ik:206: elbow_flex_joint: 2.14310886363
/check_cart_pose compute_ik:206: forearm_roll_joint: -0.615291088829
/check_cart_pose compute_ik:206: wrist_flex_joint: -0.720001579756
/check_cart_pose compute_ik:206: wrist_roll_joint: -2.57489045131
/check_cart_pose main:46: Found IK!
rosrun applications check_cart_pose.py ik 1 0 1                                                                                                                                 
/check_cart_pose main:48: No IK found.
```