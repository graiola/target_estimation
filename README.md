# Target Manager

Estimate the pose of incoming targets using a Kalman Filter.
- Targets are detected by a vision system and their pose is sent over /tf topic.
- For each detected target a KF filter is applied and the estimated pose is sent over /tf. The name of the estimated target is the same as the read one with an additional _est at the end of the name

# TODO
- [ ] start tracking after first interception
- [ ] import node to read from tf and send to robot equilibrium pose from lraiano_franka branch

# 2021/10/12
- added possibility to send estimated pose to the equilibrium pose topic of the franka robot. This will be moved into a dedicated ROS package to keep things separated.

# 2021/10/13
- tracking ok
- interecption ok
- in lraiano_franka robot control does not work -> to be checkec with this version of the code
