#!/usr/bin/env python
import roslib
roslib.load_manifest('ur_tutorial')
import rospy
from ur_msgs.msg import IOStates

rospy.init_node("echo_io_states")

def call_back(data):
  print [info.state for info in data.digital_in_states]

rospy.Subscriber('/io_states',IOStates,call_back)
rospy.spin()
