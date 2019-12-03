#pragma once

#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>

using namespace cv;

/**
 * Class is used to represent and manipulate triangles.
 */
class Triangle
{
	public:
		/**
		 * Triangle points.
		 */
		Point2f points[3];

		/**
		 * Default triangle contructor.
		 */
		Triangle()
		{
			for(int i = 0; i < 3; i++)
			{
				points[i] = Point2f(0.0, 0.0);
			}
		}

		/**
		 * Triangle contructor from corners.
		 * @param a Corner a.
		 * @param b Corner b.
		 * @param c Corner c.
		 */
		Triangle(Point2f a, Point2f b, Point2f c)
		{
			points[0] = a;
			points[1] = b;
			points[2] = c;
		}

		/**
		 * Calculate the area of this triangle.
		 * @return Area of this triangle.
		 */
		float area()
		{
			return abs(points[0].x*(points[1].y-points[2].y) + points[1].x*(points[2].y-points[0].y) + points[2].x*(points[0].y-points[1].y)) / 2.0;
		}

		/**
		 * Check if this triangle collides with another triangle.
		 * @return True if triangles are colliding, false otherwise.
		 */
		bool isColliding(Triangle t)
		{
			return containsPoint(t.points[0]) || containsPoint(t.points[1]) || containsPoint(t.points[2]) || t.containsPoint(points[0]) || t.containsPoint(points[1]) || t.containsPoint(points[2]);
		}

		/**
		 * Check if the point is inside this triangle.
		 * @param p Point to check.
		 * @return True if point is inside triangle, false otherwise.
		 */
		bool containsPoint(Point2f p)
		{	
			bool b1 = sign(p, points[0], points[1]) <= 0.0;
			bool b2 = sign(p, points[1], points[2]) <= 0.0;
			bool b3 = sign(p, points[2], points[0]) <= 0.0;

			return ((b1 == b2) && (b2 == b3));
		}

		/**
		 * Sign operation used to check if point a is before or after the line composed for b and c.
		 * @param a Point a.
		 * @param b Point b.
		 * @param c Point c.
		 * @return Float value if less than 0 point is before line.
		 */
		float sign(Point2f a, Point2f b, Point2f c)
		{
			return (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);
		}
};
