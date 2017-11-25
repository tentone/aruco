#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Triangle.cpp"

using namespace cv;
using namespace std;

/**
 * Used to represent and operate over quads.
 */
class Quadrilateral
{
	public:
		/**
		 * Vector of points that compose the quad.
		 */
		vector<Point2f> points;

		/**
		 * Quad constructor initializes 4 points.
		 */
		Quadrilateral()
		{
			for(int i = 0; i < 4; i++)
			{
				points.push_back(Point2f(0.0, 0.0));
			}
		}

		/**
		 * Quad contructor from a points.
		 */
		Quadrilateral(Point2f a, Point2f b, Point2f c, Point2f d)
		{
			points.push_back(a);
			points.push_back(b);
			points.push_back(c);
			points.push_back(d);
		}
		
		/**
		 * Calculate Area of this quad.
		 *
		 * @return Area of this quad.
		 */
		float area()
		{
			return contourArea(points);
		}

		/**
		 * Check if point is inside this quad.
		 *
		 * @param p Point to check.
		 * @return true if point is inside this quad
		 */
		bool containsPoint(Point2f p)
		{
			return pointPolygonTest(points, p, false) >= 0.0;
		}

		/**
		 * Draw quad lines to image.
		 *
		 * @param image Image where to draw line.
		 * @param color Color of the lines to be drawn.
		 * @param weight Weight of the lines.
		 */
		void draw(Mat image, Scalar color, int weigth = 1)
		{
			for(int j = 0; j < 3; j++)
			{
				line(image, points[j], points[j+1], color, weigth, 8);
			}

			line(image, points[3], points[0], color, weigth, 8);
		}

		/**
		 * Print quad info to cout.
		 */
		void print()
		{
			cout << "[" << this->points[0] << ", " << this->points[1] << ", " << this->points[2] << ", " << this->points[3] << "]" << endl;
		}

		/**
		 * Get bigger square on a vector (caller have to check vector size).
		 *
		 * @param quad Vector of quads.
		 * @return quad Bigger quad in the vector.
		 */
		static Quadrilateral biggerQuadrilateral(vector<Quadrilateral> quads)
		{
			Quadrilateral max = quads[0];
			float max_area = quads[0].area();

			//Search for bigger quad
			for(unsigned int i = 1; i < quads.size(); i++)
			{
				float area = quads[i].area();
				if(area > max_area)
				{
					max = quads[i];
					max_area = area;
				}
			}
			return max;
		}

		/**
		 * Draw all squares from vector to image.
		 *
		 * @param image Image where to draw.
		 * @param quads Vector of quads to draw.
		 * @param color Color used to draw the quads.
		 */
		static void drawVector(Mat image, vector<Quadrilateral> quads, Scalar color)
		{
			for(unsigned int i = 0; i < quads.size(); i++)
			{
				quads[i].draw(image, color);
			}
		}
};
