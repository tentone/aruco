# Aruco

 - Aruco marker detection and pose estimation.
 - Multi marker pose estimation.
 - Detection is not affected by lighting conditions.
 	- Can be used for low light marker traking.

![alt tag](https://raw.githubusercontent.com/tentone/aruco/master/images/1.png)
![alt tag](https://raw.githubusercontent.com/tentone/aruco/master/images/2.png)
![alt tag](https://raw.githubusercontent.com/tentone/aruco/master/images/3.png)

### Documentation

 - API documentation can be generated using Doxygen

### Usage ROS

 - To install in your ROS project simply copy the aruco folder into your catkin workspace and execute "catkin_make" to build the code.
 - To test with a USB camera also install usb-camera and camera-calibration from aptitude.
 - ROS Parameters
 	- Configuration
 	 	- debug
 	 	- use_opencv_coords
 	 	- cosine_limit
 	 	- theshold_block_size
 	 	- min_area
 	 	- calibrated
 	- Subscribed topics
 		- topic_camera
 		- topic_camera_info
 		- topic_marker_register
 		- topic_marker_remove
 	- Published topics
 		- topic_visible
 		- topic_position
 		- topic_rotation
 		- topic_pose

### Dependencies
 - Opencv 2.4.9+
 	- Previous versions of opencv 2 might cause problems.
 - CMake
 - ROS (indigo and later)
 	- cv-bridge

### License

 - MIT license (Available on GitHub page)

