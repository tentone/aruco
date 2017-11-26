#pragma once

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
		static Point2f refineCornerSobel(Mat gray, Point corner, int box = 10)
		{
			Rect roi = getROI(gray, corner, box);

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
				debug.at<Vec3b>(x, y) = Vec3b(0, 255, 0);
				debug.at<Vec3b>(roi.width / 2, roi.height / 2) = Vec3b(0, 0, 255);
				imshow("Corner", debug);
			#endif

			return Point2f(corner.x + x - box / 2 , corner.y + y - box / 2);
		}

		/**
		 * Corner refiment using harris operator.
		 * @param corner Initial corner position in the image.
		 * @param gray Grayscale image.
		 * @param box Box size to refine corner.
		 */
		static Point2f refineCornerHarris(Mat frame, Point corner, int box = 10)
		{
			Rect roi = getROI(frame, corner, box);
			Mat dst;
			
			int thresh = 150;

			Mat area = frame(roi).clone();
			Mat gray;
			cvtColor(area, gray, COLOR_BGR2GRAY);

			cornerHarris(gray, dst, 2, 3, 0.02);
			normalize(dst, dst, 0, 255, NORM_MINMAX);

			int x = corner.x, y = corner.y;
			int min = box;

			for(int j = 0; j < dst.rows ; j++)
			{
				for(int i = 0; i < dst.cols; i++)
				{
					if(dst.at<float>(j,i) > thresh)
					{
						int distance = sqrt(pow(j - box / 2, 2) + pow(i - box / 2, 2));
						if(distance < min)
						{
							x = j;
							y = i;
						}
						
						area.at<Vec3b>(j, i) = Vec3b(0, 255, 0);
					}
				}
			}

			area.at<Vec3b>(box / 2, box / 2) = Vec3b(0, 0, 255);
			imshow("Harris", area);

			return Point2f(corner.x + x - box / 2 , corner.y + y - box / 2);
		}

		/**
		 * Get region of interest of the image from center point and box size.
		 * @param image Image to apply ROI.
		 * @param center Center ROI point.
		 * @param box Box size.
		 */
		static Rect getROI(Mat image, Point center, int box)
		{
			Rect roi = Rect(center.x - box / 2, center.y - box / 2, box, box);
			
			if(roi.x < 0) roi.x = 0;
			if(roi.y < 0) roi.y = 0;
			if(roi.x + box > image.cols) roi.x = image.cols - box;
			if(roi.y + box > image.rows) roi.y = image.rows - box;

			return roi;
		}
};
