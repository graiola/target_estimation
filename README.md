# Target Manager

Estimate the pose of incoming targets

# ToDo

- [ ] Fix the occurence of measurementCallback_v2 -> we need to distinguish between different nodes publishing on /tf topic
- [ ] implement for multiple targets (depending on the number of /tf received)
	- [x] callback function for managing multiple targets
	- [x] update function for multiple targets
	- [x] update ros node main for multiple targets
        - [ ] add a checker in the callback to determine if the incoming measurement is related to the child frame (composition of two strings)
	- [ ] test the node with rosbag contaning multiple targets and complete the measuerementCallback_v2 function
	- [ ] automaticaly determine the number of targets from /tf messages
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

# Notes 2021/09/14
- Test ---- MeasurementCallback_v2 seems to work properly: targets are correctly stored in the maps
- created a new node: multiple_target. This correctly works with vectorial trgets (map of targets)
- to finilize the code we need to record a bag with multiple targets, understand how to determine the number of targets from incoming /tf messages ad then set n_target automatically

# Notes 2021/09/15
- Implemented new node called multiple_target_node to keep things separated from what we did considering the sigle target
- Fixed callback_v2 and update_v2
- Fixed publisher target_marker and target_estimation in multiple_target_nide
- To fix: measurementCallback_v2 so that it occurs only when a measurement is obtained. Currently, it occurs with a a frequency equal to the publishing frequency since multiple_target_node is being both publising and subscribing to /tf topic
	- [ ] check at the beginning of the callbakc which is the publishing node?
	- [ ] publish target marker to a different topic, but this may not be convenient to visualize the frames in rviz. We already do this through target_estimation topic
