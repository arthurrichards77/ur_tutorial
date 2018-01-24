#!/bin/sh
my_ip=$(ifconfig | grep -o '192\.168\.0\.1..')
export ROS_IP=$my_ip
robot_ip=192.168.0.122
export ROS_MASTER_URI=http://$robot_ip:11311/
