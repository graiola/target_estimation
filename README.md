# Target Manager

Estimate the pose of incoming targets

# Branch lraiano_multiTarget_ros_devel
- branch related to lraiano_multiTarget_ros containing the developing part which is still under development

# ToDo

- [x] Fix the occurence of measurementCallback_v2 -> only mreasurements must be fed into the KF
- [ ] implement for multiple targets (depending on the number of /tf received)
        - [x] test the node with rosbag contaning multiple targets and complete the measuerementCallback_v2 function
	- [ ] automaticaly determine the number of targets from /tf messages

# Notes 2021/09/17
- started implementin measurement callback multi -> test here how to automatically obatin the number of active targets, then move to measurement vs (upper part).

