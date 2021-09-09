# Target Manager

Estimate the pose of incoming targets

# ToDo

- [ ] implement for multiple targets (depending on the number of /tf received)
- [ ] Implement Deletion of target once they are no longer received from camera
- [ ] check covariance paametrs
- [ x ] Clean the code (gazebo stuff such as launch files and world, random)
- [ x ] implement measurement callback function (use lock()!)
- [ x ] Add target manager ROS wrapper
- [ x ] Read data (topic: rostopic echo /tf) from playback video using rosbag
- [ x ] Use the rosbag for tests


# Notes 2021/09/09

- used f=1kz which allows obtaining a smoother pose estimation
- code cleaned from unused stuff.
- used a constant number of models to test the code. Add capabilities of obtaining multiple targets on the basis of the information obtained from /tf.
