# ur_tutorial

This handout shows how to use ROS to simulate a Universal Robotics UR10 robot arm, like those mounted on the Neobotix MPO-700 platforms at BRL.  Procedures for UR5 or UR3 arms are almost identical.

##	Software Installation
The ROS drivers and simulators for the UR family of arms are provided in the universal_robot package.  We’re using ROS Melodic in the BRL PC lab in 2019 and this particular package hasn’t been upgraded since Kinetic, so you’ll need to install it from source by following these simple steps:
1.	In a terminal, navigate to the ‘src’ subdirectory of your ROS catkin workspace, which you should have set up in our first ever training session.
2.	Download the source code from Github by typing:
```git clone https://github.com/ros-industrial/universal_robot```
3.	Navigate back to the root directory of your workspace, above ‘src’, and compile it using the command below.  You may get some errors about missing ‘moveit’ content but that’s OK.  If you get any other errors, please seek help.
```catkin_make```
4.	With rospack list, check you have the “ur_gazebo” package installed.  
 
##	Simulated Arm
The “ur_gazebo” package provides a standard simulator for UR arms with the same behaviour and interface as the real arm.  Run it using:
```
roslaunch ur_gazebo ur10_joint_limited.launch
```
You should see the Gazebo simulator with a world populated only by an arm.

> Note: you’ll also get some red error messages about missing proportional gains in the controller descriptions.  You are safe to ignore these.  If you get any other errors, e.g. ‘unable to start arm_controller’ check that the ros_controllers packages as installed and that you have ‘joint_trajectory_controller’ showing in your rospack list.  If not, you'll need to do the following install: `sudo apt install joint_trajectory_controller`.  Summon help if you need admin access to a BRL machine.

Now use rqt_graph, rostopic and rosparam to have a poke around the interface.  The joint_states topic and robot_description parameter are of interest.

Create the following python script in the “scripts” subdirectory of your “ros_course” folder (or go and find it in “ur_tutorial”).  This publishes a trajectory command to the robot controller, by using the “goal” topic of its “follow joint trajectory” action.  (See ROS documentation for more detail on how to make the most of “actions”.)
 
```python
#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_course')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

rospy.init_node("simple_move_goal_pub")
pub = rospy.Publisher("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
rospy.sleep(0.5)

traj = JointTrajectory()
traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

now = rospy.get_rostime()
rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
traj.header.stamp = now

p1 = JointTrajectoryPoint()
p1.positions = [1.5, -0.2, -1.57, 0, 0 ,0]
p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p1.time_from_start = rospy.Duration(5.0)
traj.points.append(p1)

p2 = JointTrajectoryPoint()
p2.positions = [1.5, 0, -1.57, 0, 0 ,0]
p2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p2.time_from_start = rospy.Duration(10.0)
traj.points.append(p2)

p3 = JointTrajectoryPoint()
p3.positions = [2.2, 0, -1.57, 0, 0 ,0]
p3.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p3.time_from_start = rospy.Duration(15.0)
traj.points.append(p3)

ag = FollowJointTrajectoryActionGoal()
ag.goal.trajectory = traj

pub.publish(ag)
```

Run it, and you should see your simulated arm moving.  Experiment with different movements.
 
##	Visualizing with Rviz

With the robot simulator still running, start Rviz.
If it’s not already running, add a “RobotModel” widget using the “Add” button at the bottom left.  You should be able to see your UR10 robot in Rviz now.
 
Note: Rviz and Gazebo now look rather similar.  Rviz is just drawing your robot using the “joint_states” and “robot_model” information from Gazebo.  Gazebo is simulating its behaviour.

Then click the “add” button on the lower left and add a new TF viewer.  It’s easier to see what’s going on if you disable the robot model.  This is the transform tree for the robot reference frames, derived from the URDF in the “robot_model” parameter.  The “robot_state_publisher” node uses the URDF and the “joint_states” information to “tf” transforms.  You could use this information to find the pose of a target relative to the end effector.

 

