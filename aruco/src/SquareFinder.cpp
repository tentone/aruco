#pragma once

#include "math/Quadrilateral.cpp"

using namespace cv;
using namespace std;

/**
 * SquareFinder can be used to detect distorted squares in images.
 */
class SquareFinder
{
	public:
		/**
		 * Detect quads in grayscale image.
		 * @param gray Grayscale image.
		 * @param limitCosine Limit value for cosine in the quad corners, by default its 0.6.
		 * @param maxError Max error percentage relative to the square perimeter.
		 * @returns sequence of squares detected on the image the sequence is stored in the specified memory storage
		 */
		static vector<Quadrilateral> findSquares(Mat gray, double limitCosine = 0.6, int minArea = 100, double maxError = 0.025)
		{
			//Quads found
			vector<Quadrilateral> squares = vector<Quadrilateral>();

			//Contours
			vector<vector<Point>> contours;

			//Find contours and store them all as a list
			findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
			vector<Point> approx;

			for(unsigned int i = 0; i < contours.size(); i++)
			{
				//Approximate contour with accuracy proportional to the contour perimeter
				approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * maxError, true);

				//Square contours have 4 vertices after approximation relatively large area (to filter out noisy contours)and be convex.
				if(approx.size() == 4 && fabs(contourArea(Mat(approx))) > minArea && isContourConvex(Mat(approx)))
				{
					float maxCosine = 0;

					//Find the maximum cosine of the angle between joint edges
					for(int j = 2; j < 5; j++)
					{
						float cosine = fabs(angleCornerPointsCos(approx[j%4], approx[j-2], approx[j-1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					//Check if all angle corner close to 90 (more than the max cosine)
					if(maxCosine < limitCosine)
					{
						Quadrilateral quad = Quadrilateral();
						
						for(int j = 0; !approx.empty() && j < 4; j++)
						{
							quad.points[j] = approx.back();
							approx.pop_back();
						}

						squares.push_back(quad);
					}
				}
			}

			return squares;
		}

		/**
		 * Draw quads into the matrix.
		 * 
		 * @param mat Mat to draw quads.
		 * @param quads Vector of quadrilaterals to draw.
		 */
		static void drawQuads(Mat mat, vector<Quadrilateral> quads)
		{
			for(unsigned int i = 0; i < quads.size(); i++)
			{
				for(unsigned int j = 0; j < 4; j++)
				{
					line(mat, quads[i].points[j], quads[i].points[(j + 1) % 4], Scalar(255, 0, 255), 2);
				}
			}
		}

		/**
		 * Finds a cosine of angle between vectors from pt0->pt1 and from pt0->pt2.
		 *
		 * @param b Point b.
		 * @param c Point c.
		 * @param a Point a.
		 */
		static float angleCornerPointsCos(Point b, Point c, Point a)
		{
			float dx1 = b.x - a.x;
			float dy1 = b.y - a.y;
			float dx2 = c.x - a.x;
			float dy2 = c.y - a.y;

			return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12);
		}
};
