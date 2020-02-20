# ur_tutorial
A tutorial package for simulating the UR10 arm in ROS and Gazebo.

To use in ROS Melodic:
1. If you've not got ROS up and running already, start with a full ROS Melodic install, i.e. `sudo apt install ros-melodic-desktop-full` following instructions from (http://wiki.ros.org/melodic/Installation/Ubuntu) and set up a workspace according to (http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
2. Manually install the `joint_trajectory_controller` package using `sudo apt install ros-melodic-joint-trajectory-controller`
3. In your workspace `src` directory, clone the ROS UR package: `git clone https://github.com/ros-industrial/universal_robot.git`.  Resist the urge to do a catkin_make as it's likely to throw many errors.
4. Clone this `ur_tutorial` repository there as well.  You should be able to see `ur_tutorial` and `ur_gazebo` in `rospack list`.
5. Run `roslaunch ur_gazebo ur10.launch` and you should see a simulation start.  Errors about missing p gains are expected.
5. Run any one of the Python scripts in the `scripts` folder of `ur_tutorial`, e.g. `rosrun ur_tutorial simple_move.py`, and you should see the simulated arm do a little move.

Apologies this is so complicated.  UR robot drivers seem to have got caught in dependency problems, with the old Simulator package having been left behind by recent ROS updates.

> This package is *only* intended for running simulations in Melodic.  The actual arm drivers loaded here are unlikely to work.  New UR drivers are available but they don't provide the nice simulator wrapper.
