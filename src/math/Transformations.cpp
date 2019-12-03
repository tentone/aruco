#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/**
 * Transformations class contains method to help create and apply tranformations to points.
 */
class Transformations
{
	public:
		/**
		 * Creates a new rotation matrix from euler rotation.
		 * @return Rotation matrix created from euler rotation.
		 */
		static Mat rotationMatrix(Point3d euler)
		{
			//Rotation on x axis
			Mat rx = (Mat_<double>(3,3) <<
			1, 0, 0,
			0, cos(euler.x), -sin(euler.x),
			0, sin(euler.x), cos(euler.x));

			//Rotation on y axis
			Mat ry = (Mat_<double>(3,3) <<
			cos(euler.y), 0, sin(euler.y),
			0, 1, 0,
			-sin(euler.y), 0, cos(euler.y));

			//Rotation on z axis
			Mat rz = (Mat_<double>(3,3) <<
			cos(euler.z), -sin(euler.z), 0,
			sin(euler.z), cos(euler.z), 0,
			0, 0, 1);

			return  rz * ry * rx;
		}
};
