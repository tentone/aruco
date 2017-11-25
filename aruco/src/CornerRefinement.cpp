#pragma once

#define DEBUG false

/**
 * This class contains methods to perform corner refinement on the points found by the SquareFinder.
 */
class CornerRefinement
{	
	public:
		/**
		 * Refine corner position using a grayscale version of the captured image (without threshold).
		 * This method uses the sobel operator and assumes that only one corner is visible, if the box size is too large corners from the marker migth be visible.
		 * @param corner Initial corner position in the image.
		 * @param gray Grayscale image.
		 * @param box Box size to refine corner.
		 */
		static Point2f refineCornerSobel(Point corner, Mat gray, int box)
		{
			Rect roi = Rect(corner.x - box / 2, corner.y - box / 2, box, box);
			
			if(roi.x < 0) roi.x = 0;
			if(roi.y < 0) roi.y = 0;
			if(roi.x + box > gray.cols) roi.x = gray.cols - box;
			if(roi.y + box > gray.rows) roi.y = gray.rows - box;

			Mat sobel;
			Sobel(gray(roi), sobel, -1, 2, 2, 3, 5);

			int rows = sobel.rows;
			int cols = sobel.cols;

			int x = 0, y = 0;
			int max = sobel.data[0];

			for(int i = 0; i < rows; i++)
			{
				for(int j = 0; j < cols; j++)
				{
					int value = sobel.data[rows * i + j];
					if(value > max)
					{
						x = j;
						y = i;
						max = value;
					}
				}
			}

			#if DEBUG == true
				Mat debug = gray(roi).clone();
				circle(debug, Point(x, y), 1, Scalar(0, 255, 0), -1);
				imshow("Corner", debug);
				imshow("Sobel", sobel);
			#endif

			return Point2f(corner.x + x - box / 2 , corner.y + y - box / 2);
		}

		static Point2f refineCornerHarris(Point corner, Mat gray, int box)
		{
			Rect roi = Rect(corner.x - box / 2, corner.y - box / 2, box, box);
			
			if(roi.x < 0) roi.x = 0;
			if(roi.y < 0) roi.y = 0;
			if(roi.x + box > gray.cols) roi.x = gray.cols - box;
			if(roi.y + box > gray.rows) roi.y = gray.rows - box;

			//TODO <ADD CODE HERE>

			return Point2f(corner.x, corner.y);
		}
};
