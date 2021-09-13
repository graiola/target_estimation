# Target Manager

Estimate the pose of incoming targets

# ToDo

- [ ] implement for multiple targets (depending on the number of /tf received)
	- [x] callback function for managing multiple targets
	- [ ] update function for multiple targets
	- [ ] update ros node main for multiple targets
- [ ] Implement Deletion of target once they are no longer received from camera
- [ ] check covariance paametrs
- [x] add launch rviz from launch file
- [x] Clean the code (gazebo stuff such as launch files and world, random)
- [x] implement measurement callback function (use lock()!)
- [x] Add target manager ROS wrapper
- [x] Read data (topic: rostopic echo /tf) from playback video using rosbag
- [x] Use the rosbag for tests


# Notes 2021/09/09

- used f=1kHz which allows obtaining a smoother pose estimation
- code cleaned from unused stuff.
- used a constant number of models to test the code. Add capabilities of obtaining multiple targets on the basis of the information obtained from /tf.

# Notes 2021/09/10

- added rviz launch with configs set in launch file
- started coding for multiple targets -> I used the map and a vector of target id read from the /tf. Nevertheless, the code cannot properly initialize the KF.

# Notes 2021/09/13
- code with single target pushed to branch lraiano_test_ros
- created new branch for developing code to manage multiple tagets (lraiano_test_ros_dev)
- implemented measurementCallbck_v2 to manage multiple incoming targets. NB: to be tested
- defined update function v2
- put FIXME parts in node main function (target_node.cpp)
