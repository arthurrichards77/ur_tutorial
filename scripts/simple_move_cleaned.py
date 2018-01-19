#!/usr/bin/env python
import roslib
roslib.load_manifest('ur_tutorial')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient
import math

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class URMover:
	def __init__(self):
		rospy.init_node("simple_traj")
		self.client = SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
		self.goal = FollowJointTrajectoryGoal()
		self.goal.trajectory = JointTrajectory()
		self.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

	def connect_action(self):
		print "waiting to connect..."
		self.client.wait_for_server()
		print "connected! "


	def add_point(self, time_from_start, positions, velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
		assert(len(positions)==6)
		p = JointTrajectoryPoint()
		p.positions = positions
		p.velocities = velocities
		p.time_from_start = time_from_start
		#self.goal.trajectory.points.append(p)
		self.goal.trajectory.points = [p]

	def execute(self):
		self.client.send_goal(self.goal)
		print "sent the goal"

	def wait_for_completion(self):
		print "waiting to get there"
		self.client.wait_for_result()
		print "got there"

mover = URMover()

def callback(data):
	print data.data
	if data.data == "A":
		print "do A"
		moveA()
	elif data.data == "B":
		print "do B"
		moveB()



#def callback2(data):
	#print data.position
	#print data.position[3]



def moveA():
	global mover
	#mover = URMover()
	#mover.connect_action()
	#straight up

	position = [0, -math.pi/2, 0, 0, 0 ,0]
	
	position = makeStraight(position)

	mover.add_point(rospy.Duration(2.0), position)
	mover.execute()
	mover.wait_for_completion()
	

def moveB():
	global mover
	#bend joint 3
#	mover.add_point(rospy.Duration(2.0), [0, -math.pi/2, math.pi/4, 0, 0 ,0])

	position = [0, -math.pi/2, math.pi/4, 0, 0 ,0]
	position = makeStraight(position)
	mover.add_point(rospy.Duration(2.0), position)
	mover.execute()
	mover.wait_for_completion()

def makeStraight(position):
	
	angle = position[1] + position[2]

	position[3] = 0 - angle - math.pi/2

	return position



def main():
	
	mover.connect_action()

	rospy.Subscriber("moveArm", String, callback)
	#rospy.Subscriber("joint_states", JointState, callback2)

	

	rospy.spin()


if __name__ == '__main__': main()

