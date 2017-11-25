#pragma once

#include <string>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>

#include "math/Transformations.cpp"

using namespace cv;
using namespace std;

/**
 * Class is used to store info about a marker.
 * The info present in this class is related with real world information.
 * Units are store in meters for positions and dimensions and radians for angles.
 */
class ArucoMarkerInfo
{
	public:
		/**
		 * Marker ID.
		 */
		int id;

		/**
		 * Size of the marker in real position scale.
		 */
		double size;

		/**
		 * Marker position in real world coordinates.
		 */
		Point3d position;

		/**
		 * Euler rotation of this marker.
		 */
		Point3d rotation;

		/**
		 * World corner points calculated from position and rotation of the marker.
		 * This vector should have only 4 points.
		 * But can be used to store extra known points for the marker.
		 * World points stored in the following order: TopLeft, TopRight, BottomRight, BottomLeft.
		 */
		vector<Point3f> world;

		/**
		 * Default constructor with id -1.
		 */
		ArucoMarkerInfo()
		{
			id = -1;
			size = 1.0;
			position = Point3d(0.0, 0.0, 0.0);
			rotation = Point3d(0.0, 0.0, 0.0);
			world = vector<Point3f>();
			calculateWorldPoints();
		}

		/**
		 * Aruco marker constructor.
		 * @param id Marker id.
		 * @param size Marker size in meters.
		 * @param position Marker world position.
		 */
		ArucoMarkerInfo(int _id, double _size, Point3f _position)
		{
			id = _id;
			size = _size;
			position = _position;
			rotation = Point3f(0.0, 0.0, 0.0);
			world = vector<Point3f>();
			calculateWorldPoints();
		}

		/**
		 * Aruco marker constructor.
		 * @param id Marker id.
		 * @param size Marker size in meters.
		 * @param position Marker world position.
		 * @param rotation Marker world euler rotation.
		 */
		ArucoMarkerInfo(int _id, double _size, Point3f _position, Point3f _rotation)
		{
			id = _id;
			size = _size;
			position = _position;
			rotation = _rotation;
			world = vector<Point3f>();
			calculateWorldPoints();
		}

		/**
		 * Calculate the marker world points, considering the marker center position and rotation.
		 * First the marker is rotated and is translated after so the rotation is always relative to the marker center.
		 */
		void calculateWorldPoints()
		{
			double half = size / 2.0;

			world.clear();

			world.push_back(Point3f(-half, -half, 0));
			world.push_back(Point3f(-half, +half, 0));
			world.push_back(Point3f(half, +half, 0));
			world.push_back(Point3f(half, -half, 0));

			Mat rot = Transformations::rotationMatrix(rotation);
			
			for(unsigned int i = 0; i < world.size(); i++)
			{
				Mat temp = (Mat_<double>(3, 1) << world[i].x, world[i].y, world[i].z);
				Mat transf = rot * temp;
				
				world[i].x = transf.at<double>(0, 0) + position.x;
				world[i].y = transf.at<double>(1, 0) + position.y;
				world[i].z = transf.at<double>(2, 0) - position.z;
			}
		}

		/**
		 * Printo info of this marker to the stdout.
		 */
		void print()
		{
			cout << "{" << endl;
			cout << "    ID: " << id << endl;
			cout << "    Size: " << size << endl;
			cout << "    Position: " << position.x << ", " << position.y << ", " << position.z << endl;
			cout << "    Rotation: " << rotation.x << ", " << rotation.y << ", " << rotation.z << endl;

			for(unsigned int i = 0; i < world.size(); i++)
			{
				cout << "    World: " << world[i].x << ", " << world[i].y << ", " << world[i].z << endl;
			}

			cout << "}" << endl;
		}
};
