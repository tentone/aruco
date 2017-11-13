#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/image_encodings.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "aruco/Marker.h"

#include "../ArucoMarker.cpp"
#include "../ArucoMarkerInfo.cpp"
#include "../ArucoDetector.cpp"

using namespace cv;
using namespace std;

/**
 * Camera calibration matrix pre initialized with calibration values for the test camera.
 */
double data_calibration[9] = {570.3422241210938, 0, 319.5, 0, 570.3422241210938, 239.5, 0, 0, 1};
Mat calibration;

/**
 * Lenses distortion matrix initialized with values for the test camera.
 */
double data_distortion[5] = {0, 0, 0, 0, 0};
Mat distortion;

/**
 * List of known of markers, to get the absolute position and rotation of the camera, some of these are required.
 */
vector<ArucoMarkerInfo> known = vector<ArucoMarkerInfo>();

/**
 * ROS node visibility publisher.
 * Publishes true when a known marker is visible, publishes false otherwise.
 */
ros::Publisher pub_visible;

/**
 * ROS node position publisher.
 */
ros::Publisher pub_position;

/**
 * ROS node rotation publisher.
 */
ros::Publisher pub_rotation;

/**
 * ROS node pose publisher.
 */
ros::Publisher pub_pose;

/**
 * Pose publisher sequence counter.
 */
int pub_pose_seq = 0;

/**
 * Flag to check if calibration parameters were received.
 * If set to false the camera will be calibrated when a camera info message is received.
 */
bool calibrated;

/**
 * Flag to determine if OpenCV or ROS coordinates are used.
 */
bool use_opencv_coords;

/**
 * When debug parameter is se to true the node creates a new cv window to show debug information.
 * By default is set to false.
 * If set true the node will open a debug window.
 */
bool debug;

/**
 * Cosine limit used during the quad detection phase.
 * Value between 0 and 1.
 * By default 0.8 is used.
 * The bigger the value more distortion tolerant the square detection will be.
 */
float cosine_limit;

/**
 * Adaptive theshold pre processing block size.
 */
int theshold_block_size;

/**
 * Minimum threshold block size.
 * By default 5 is used.
 */
int theshold_block_size_min;


/**
 * Maximum threshold block size.
 * By default 9 is used.
 */
int theshold_block_size_max;

/**
 * Minimum area considered for aruco markers.
 * Should be a value high enough to filter blobs out but detect the smallest marker necessary.
 * By default 100 is used.
 */
int min_area;

/**
 * Draw yellow text with black outline into a frame.
 * @param frame Frame mat.
 * @param text Text to be drawn into the frame.
 * @param point Position of the text in frame coordinates.
 */
void drawText(Mat frame, string text, Point point)
{
	putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2, CV_AA);
	putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1, CV_AA);
}

/**
 * Callback executed every time a new camera frame is received.
 * This callback is used to process received images and publish messages with camera position data if any.
 */
void onFrame(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		
		Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

		//Process image and get markers
		vector<ArucoMarker> markers = ArucoDetector::getMarkers(frame, cosine_limit, theshold_block_size, min_area);

		//Visible
		vector<ArucoMarker> found;

		//Vector of points
		vector<Point2f> projected;
		vector<Point3f> world;

		if(markers.size() == 0)
		{
			theshold_block_size += 2;

			if(theshold_block_size > theshold_block_size_max)
			{
				theshold_block_size = theshold_block_size_min;
			}
		}

		//Check known markers and build known of points
		for(unsigned int i = 0; i < markers.size(); i++)
		{
			for(unsigned int j = 0; j < known.size(); j++)
			{
				if(markers[i].id == known[j].id)
				{
					markers[i].attachInfo(known[j]);
					
					for(unsigned int k = 0; k < 4; k++)
					{
						projected.push_back(markers[i].projected[k]);
						world.push_back(known[j].world[k]);
					}

					found.push_back(markers[i]);
				}
			}
		}

		//Draw markers
		if(debug)
		{
			ArucoDetector::drawMarkers(frame, markers, calibration, distortion);
		}

		//Check if any marker was found
		if(world.size() > 0)
		{
			//Calculate position and rotation
			Mat rotation, position;

			#if CV_MAJOR_VERSION == 2
				solvePnP(world, projected, calibration, distortion, rotation, position, false, ITERATIVE);
			#else
				solvePnP(world, projected, calibration, distortion, rotation, position, false, SOLVEPNP_ITERATIVE);
			#endif

			//Invert position and rotation to get camera coords
			Mat rodrigues;
			Rodrigues(rotation, rodrigues);
			
			Mat camera_rotation;
			Rodrigues(rodrigues.t(), camera_rotation);
			
			Mat camera_position = -rodrigues.t() * position;

			//Publish position and rotation
			geometry_msgs::Point message_position, message_rotation;

			//Opencv coordinates
			if(use_opencv_coords)
			{
				message_position.x = camera_position.at<double>(0, 0);
				message_position.y = camera_position.at<double>(1, 0);
				message_position.z = camera_position.at<double>(2, 0);
				
				message_rotation.x = camera_rotation.at<double>(0, 0);
				message_rotation.y = camera_rotation.at<double>(1, 0);
				message_rotation.z = camera_rotation.at<double>(2, 0);
			}
			//Robot coordinates
			else
			{
				message_position.x = camera_position.at<double>(2, 0);
				message_position.y = -camera_position.at<double>(0, 0);
				message_position.z = -camera_position.at<double>(1, 0);
				
				message_rotation.x = camera_rotation.at<double>(2, 0);
				message_rotation.y = -camera_rotation.at<double>(0, 0);
				message_rotation.z = -camera_rotation.at<double>(1, 0);
			}

			pub_position.publish(message_position);
			pub_rotation.publish(message_rotation);

			//Publish pose
			geometry_msgs::PoseStamped message_pose;

			//Header
			message_pose.header.frame_id = "aruco";
			message_pose.header.seq = pub_pose_seq++;
			message_pose.header.stamp = ros::Time::now();

			//Position
			message_pose.pose.position.x = message_position.x;
			message_pose.pose.position.y = message_position.y;
			message_pose.pose.position.z = message_position.z;

			//Convert to quaternion
			double x = message_rotation.x;
			double y = message_rotation.y;
			double z = message_rotation.z;

			//Module of angular velocity
			double angle = sqrt(x*x + y*y + z*z);

			if(angle > 0.0)
			{
				message_pose.pose.orientation.x = x * sin(angle/2.0)/angle;
				message_pose.pose.orientation.y = y * sin(angle/2.0)/angle;
				message_pose.pose.orientation.z = z * sin(angle/2.0)/angle;
				message_pose.pose.orientation.w = cos(angle/2.0);
			}
			//To avoid illegal expressions
			else
			{
				message_pose.pose.orientation.x = 0.0;
				message_pose.pose.orientation.y = 0.0;
				message_pose.pose.orientation.z = 0.0;
				message_pose.pose.orientation.w = 1.0;
			}
			
			pub_pose.publish(message_pose);

			//Debug
			if(debug)
			{
				ArucoDetector::drawOrigin(frame, found, calibration, distortion, 0.1);
				
				drawText(frame, "Position: " + to_string(message_position.x) + ", " + to_string(message_position.y) + ", " + to_string(message_position.z), Point2f(10, 160));
				drawText(frame, "Rotation: " + to_string(message_rotation.x) + ", " + to_string(message_rotation.y) + ", " + to_string(message_rotation.z), Point2f(10, 180));
			}
		}
		else if(debug)
		{
			drawText(frame, "Position: unknown", Point2f(10, 160));
			drawText(frame, "Rotation: unknown", Point2f(10, 180));
		}

		//Publish visible
		std_msgs::Bool message_visible;
		message_visible.data = world.size() != 0;
		pub_visible.publish(message_visible);

		//Debug info
		if(debug)
		{
			drawText(frame, "Aruco ROS Debug", Point2f(10, 20));
			drawText(frame, "OpenCV V" + to_string(CV_MAJOR_VERSION) + "." + to_string(CV_MINOR_VERSION), Point2f(10, 40));
			drawText(frame, "Cosine Limit (A-Q): " + to_string(cosine_limit), Point2f(10, 60));
			drawText(frame, "Threshold Block (W-S): " + to_string(theshold_block_size), Point2f(10, 80));
			drawText(frame, "Min Area (E-D): " + to_string(min_area), Point2f(10, 100));
			drawText(frame, "Visible: " + to_string(message_visible.data), Point2f(10, 120));
			drawText(frame, "Calibrated: " + to_string(calibrated), Point2f(10, 140));
			
			imshow("Aruco", frame);

			char key = (char) waitKey(1);

			if(key == 'q')
			{
				cosine_limit += 0.05;
			}
			else if(key == 'a')
			{
				cosine_limit -= 0.05;
			}

			if(key == 'w')
			{
				theshold_block_size += 2;
			}
			else if(key == 's' && theshold_block_size > 3)
			{
				theshold_block_size -= 2;
			}

			if(key == 'e')
			{
				min_area += 50;
			}
			else if(key == 'd')
			{
				min_area -= 50;
			}
		}
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("Error getting image data");
	}
}

/**
 * On camera info callback.
 * Used to receive camera calibration parameters.
 */
void onCameraInfo(const sensor_msgs::CameraInfo &msg)
{
	if(!calibrated)
	{
		calibrated = true;
		
		for(unsigned int i = 0; i < 9; i++)
		{
			calibration.at<double>(i / 3, i % 3) = msg.K[i];
		}
		
		for(unsigned int i = 0; i < 5; i++)
		{
			distortion.at<double>(0, i) = msg.D[i];
		}

		if(debug)
		{
			cout << "Camera calibration param received" << endl;
			cout << "Camera: " << calibration << endl;
			cout << "Distortion: " << distortion << endl;
		}
	}
}

/**
 * Callback to register markers on the marker list.
 * This callback received a custom marker message.
 */
void onMarkerRegister(const aruco::Marker &msg)
{
	for(unsigned int i = 0; i < known.size(); i++)
	{
		if(known[i].id == msg.id)
		{
			known.erase(known.begin() + i);
			cout << "Marker " << to_string(msg.id) << " already exists, was replaced." << endl;
			break;
		}
	}

	known.push_back(ArucoMarkerInfo(msg.id, msg.size, Point3d(msg.posx, msg.posy, msg.posz), Point3d(msg.rotx, msg.roty, msg.rotz)));
	cout << "Marker " << to_string(msg.id) << " added." << endl;
}

/**
 * Callback to remove markers from the marker list.
 * Markers are removed by publishing the remove ID to the remove topic.
 */
void onMarkerRemove(const std_msgs::Int32 &msg)
{
	for(unsigned int i = 0; i < known.size(); i++)
	{
		if(known[i].id == msg.data)
		{
			known.erase(known.begin() + i);
			cout << "Marker " << to_string(msg.data) << " removed." << endl;
			break;
		}
	}
}

/**
 * Converts a string with numeric values separated by a delimiter to an array of double values.
 * If 0_1_2_3 and delimiter is _ array will contain {0, 1, 2, 3}.
 * @param data String to be converted
 * @param values Array to store values on
 * @param cout Number of elements in the string
 * @param delimiter Separator element
 * @return Array with values.
 */
void stringToDoubleArray(string data, double* values, unsigned int count, string delimiter)
{
	unsigned int pos = 0, k = 0;

	while((pos = data.find(delimiter)) != string::npos && k < count)
	{
		string token = data.substr(0, pos);
		values[k] = stod(token);
		data.erase(0, pos + delimiter.length());
		k++;
	}
}

/**
 * Main method launches aruco ros node, the node gets image and calibration parameters from camera, and publishes position and rotation of the camera relative to the markers.
 * Units should be in meters and radians, the markers are described by a position and an euler rotation.
 * Position is also available as a pose message that should be easier to consume by other ROS nodes.
 * Its possible to pass markers as arugment to this node or register and remove them during runtime using another ROS node.
 * The coordinate system used by OpenCV uses Z+ to represent depth, Y- for height and X+ for lateral, but for the node the coordinate system used is diferent X+ for depth, Z+ for height and Y- for lateral movement.
 * The coordinates are converted on input and on output, its possible to force the OpenCV coordinate system by setting the use_opencv_coords param to true.
 *   
 *           ROS          |          OpenCV
 *    Z+                  |    Y- 
 *    |                   |    |
 *    |    X+             |    |    Z+
 *    |    /              |    |    /
 *    |   /               |    |   /
 *    |  /                |    |  /
 *    | /                 |    | /
 *    |/                  |    |/
 *    O-------------> Y-  |    O-------------> X+
 *
 * @param argc Number of arguments.
 * @param argv Value of the arguments.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco");

	//Private and global node instances
	ros::NodeHandle node("aruco");
	ros::NodeHandle global;
	
	//Parameters
	node.param<bool>("debug", debug, false);
	node.param<bool>("use_opencv_coords", use_opencv_coords, false);
	node.param<float>("cosine_limit", cosine_limit, 0.8);
	node.param<int>("theshold_block_size_min", theshold_block_size_min, 3);
	node.param<int>("theshold_block_size_max", theshold_block_size_max, 21);
	node.param<int>("min_area", min_area, 100);
	node.param<bool>("calibrated", calibrated, true);

	//Initial threshold block size
	theshold_block_size = (theshold_block_size_min + theshold_block_size_max) / 2;
	if(theshold_block_size % 2 == 0)
	{
		theshold_block_size++;
	}

	//Initialize calibration matrices
	calibration = Mat(3, 3, CV_64F, data_calibration);
	distortion = Mat(1, 5, CV_64F, data_distortion);

	//Camera instrinsic calibration parameters
	if(node.hasParam("calibration"))
	{
		string data;
		node.param<string>("calibration", data, "");
		
		double values[9];
		stringToDoubleArray(data, values, 9, "_");

		for(unsigned int i = 0; i < 9; i++)
		{
			calibration.at<double>(i / 3, i % 3) = values[i];
		}

		calibrated = true;
	}

	//Camera distortion calibration parameters
	if(node.hasParam("distortion"))
	{
		string data;
		node.param<string>("distortion", data, "");	

		double values[5];
		stringToDoubleArray(data, values, 5, "_");

		for(unsigned int i = 0; i < 5; i++)
		{
			distortion.at<double>(0, i) = values[i];
		}

		calibrated = true;
	}

	//Aruco makers passed as parameters
	for(unsigned int i = 0; i < 1024; i++)
	{
		if(node.hasParam("marker" + to_string(i)))
		{
			string data;
			node.param<string>("marker" + to_string(i), data, "1_0_0_0_0_0_0");

			double values[7];
			stringToDoubleArray(data, values, 7, "_");

			//Use OpenCV coordinates
			if(use_opencv_coords)
			{
				known.push_back(ArucoMarkerInfo(i, values[0], Point3d(values[1], values[2], values[3]), Point3d(values[4], values[5], values[6])));
			}
			//Convert coordinates (-Y, -Z, +X)
			else
			{
				known.push_back(ArucoMarkerInfo(i, values[0], Point3d(-values[2], -values[3], -values[1]), Point3d(-values[5], -values[6], values[4])));
			}
		}
	}

	//Print all known markers
	if(debug)
	{
		for(unsigned int i = 0; i < known.size(); i++)
		{
			known[i].print();
		}
	}

	//Subscribed topic names
	string topic_camera, topic_camera_info, topic_marker_register, topic_marker_remove;
	node.param<string>("topic_camera", topic_camera, "/rgb/image");
	node.param<string>("topic_camera_info", topic_camera_info, "/rgb/camera_info");
	node.param<string>("topic_marker_register", topic_marker_register, "/marker_register");
	node.param<string>("topic_marker_remove", topic_marker_register, "/marker_remove");

	//Publish topic names
	string topic_visible, topic_position, topic_rotation, topic_pose;
	node.param<string>("topic_visible", topic_visible, "/visible");
	node.param<string>("topic_position", topic_position, "/position");
	node.param<string>("topic_rotation", topic_rotation, "/rotation");
	node.param<string>("topic_pose", topic_pose, "/pose");

	//Advertise topics
	pub_visible = global.advertise<std_msgs::Bool>(node.getNamespace() + topic_visible, 10);
	pub_position = global.advertise<geometry_msgs::Point>(node.getNamespace() + topic_position, 10);
	pub_rotation = global.advertise<geometry_msgs::Point>(node.getNamespace() + topic_rotation, 10);
	pub_pose = global.advertise<geometry_msgs::PoseStamped>(node.getNamespace() + topic_pose, 10);

	//Subscribe topics
	image_transport::ImageTransport it(global);
	image_transport::Subscriber sub_camera = it.subscribe(topic_camera, 1, onFrame);
	ros::Subscriber sub_camera_info = global.subscribe(topic_camera_info, 1, onCameraInfo);
	ros::Subscriber sub_marker_register = global.subscribe(topic_marker_register, 1, onMarkerRegister);
	ros::Subscriber sub_marker_remove = global.subscribe(topic_marker_remove, 1, onMarkerRemove);

	ros::spin();

	return 0;
}

