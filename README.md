# Aruco

 - Aruco marker detection and pose estimation.
 - Multi marker pose estimation.
 - Detection is not affected by lighting conditions.
 	- Can be used for low light marker traking.

### Documentation

 - API documentation can be generated using Doxygen

### Usage ROS

 - To install in your ROS project simply copy the aruco folder into your catkin workspace and execute "catkin_make" to build the code.
 - To test with a USB camera also install usb-camera and camera-calibration from aptitude.
 - ROS Parameters
 	- Configuration
 	 	- debug
 	 		- When debug parameter is se to true the node creates a new cv window to show debug information.
 	 		- Default false
 	 	- use_opencv_coords
 	 		- When set opencv coordinates are used, otherwise wiigo coords are used (X+ depth, Z+ height, Y+ lateral)
 	 		- Default false
 	 	- cosine_limit
 	 		- Cosine limit used during the quad detection phase. By default 0.8 is used. The bigger the value more distortion tolerant the square detection will be.
 	 		- Default 0.7
 	 	- theshold_block_size
 	 		- Adaptive theshold base block size.
 	 		- Default 9
 	 	- min_area
 	 		- Minimum area considered for aruco markers. Should be a value high enough to filter blobs out but detect the smallest marker necessary.
 	 		- Default 100
 	 	- calibrated
 	 		- Used to indicate if the camera should be calibrated using external message of use default calib parameters
 	 		- Default true
 	 	- calibration
 	 		- Camera intrinsic calibration matrix as defined by opencv (values by row separated by _ char)
 	 		- Ex "260.3_0_154.6_0_260.5_117_0_0_1"
 	 	- distortion
 	 		- Camera distortion matrix as defined by opencv composed of up to 5 parameters (values separated by _ char)
 	 		- Ex "0.007_-0.023_-0.004_-0.0006_-0.16058"
 	 	- marker###
 	 		- These parameters are used to pass to the node a list of known markers, these markers will be used to calculate the camera pose in the world.
 	 		- Markers are declared in the format marker###: "<size>_<posx>_<posy>_<posz>_<rotx>_<roty>_<rotz>"
			- Ex marker768 "0.156_0_0_0_0_0_0"
 	- Subscribed topics
 		- topic_camera
 			- Camera image topic
 			- Default "/camera/rgb/image_raw"
 		- topic_camera_info
 			- Camera info_expects a CameraInfo message
 			- Default "/camera/rgb/camera_info"
 		- topic_marker_register
 			- Register markers in the node
 			- Default "/marker_register"
 		- topic_marker_remove
 			- Remove markers registered in the node
 			- Default "/marker_remove"
 	- Published topics
 		- topic_visible
 			- Publishes true when a marker is visible_false otherwise
 			- Default "/visible"
 		- topic_position
 			- Publishes the camera world position relative to the registered markers as a Point message
 			- Default "/position"
 		- topic_rotation
 			- Publishes the camera world rotation relative to the registered markers
 			- Default "/rotation"
 		- topic_pose
 			- Publishes camera rotation and position as Pose message
 			- Default "/pose"

### Dependencies
 - Opencv 2.4.9+
 	- Previous versions of opencv 2 might cause problems.
 - CMake
 - ROS (indigo and later)
 	- cv-bridge

### License

 - MIT license (Available on GitHub page)

