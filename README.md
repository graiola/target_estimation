# Target Manager

- Branch Name: lraiano_catching_target_devel

Estimate the pose of incoming targets using a Kalman Filter.
- Targets are detected by a vision system and their pose is sent over /tf topic.
- For each detected target a KF filter is applied and the estimated pose is sent over /tf. The name of the estimated target is the same as the read one with an additional _est at the end of the name
- If set, it is possible to send an equilibrium pose to a robot over robot_equlibrium_pose_topic (which should then be renamed according to the specific robot used)

# TODO
- [ ] add camera to robot transformation
- [ ] implement closest target interception to track only one target per time when multiple targets are detected
- [ ] start tracking after first interception of the closest target
- [ ] remove rt_logger dependency after completion on branch
- [ ] delete brach lraiano_franka_integration_devel beacouse the woking parts have been moved to lraiano_catching_targets
- [x] start tracking after first interception
- [x] import node to read from tf and send to robot equilibrium pose from lraiano_franka branch

# 2021/10/18
- interception then start tracking ok
- get target id of the one which requires the least time for the robot to catch. Anyway, I need to fix the part that allows obtaining also its name
