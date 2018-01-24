#!/usr/bin/env python
import roslib
roslib.load_manifest('ur_tutorial')
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
