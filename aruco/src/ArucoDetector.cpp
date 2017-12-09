#pragma once

#include <string>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include "SquareFinder.cpp"
#include "CornerRefinement.cpp"
#include "ArucoMarker.cpp"
#include "ArucoMarkerInfo.cpp"

#define DEBUG false

using namespace cv;
using namespace std;

/**
 * Class is used to detect 5x5 aruco markers.
 * It contains method for detection, data reading, previsualization and debugging aruco markers information.
 * After detection camera position relative to the aruco markers can be easily calculated using opencv solvePnP.
 */
class ArucoDetector
{	
	public:
		/**
		 * Process image to identify aruco markers.
		 * Applies pre-processing over the frame and get list of quads in the frame.
		 * @param frame Frame to be processed.
		 * @param limitCosine Higher values allow detection of more distorted markers but performance is slower
		 */
		static vector<ArucoMarker> getMarkers(Mat frame, float limitCosine = 0.7, int thresholdBlockSize = 7, int minArea = 100, double maxError = 0.025)
		{
			//Create a grayscale image
			Mat gray;
			cvtColor(frame, gray, COLOR_BGR2GRAY);

			//Adaptive threshold
			Mat thresh;
			adaptiveThreshold(gray, thresh, 255, THRESH_BINARY, ADAPTIVE_THRESH_MEAN_C, thresholdBlockSize, 0.0);

			#if DEBUG
				imshow("Adaptive", thresh);
			#endif

			//Get quads
			vector<Quadrilateral> quads = SquareFinder::findSquares(thresh, limitCosine, minArea, maxError);

			#if DEBUG
				Mat quad = frame.clone();
				SquareFinder::drawQuads(quad, quads);
				imshow("Quads", quad);
			#endif

			//List of markers
			vector<ArucoMarker> markers = vector<ArucoMarker>();

			//Transform quads and filter invalid markers
			for(unsigned int i = 0; i < quads.size(); i++)
			{

				Mat board = deformQuad(frame, Point2i(49, 49), quads[i].points);
				Mat binary = processArucoImage(board);

				//Process aruco image and get data
				ArucoMarker marker = readArucoData(binary);
				marker.projected = quads[i].points;

				//Check if marker is valid
				if(marker.validate())
				{					
					//Show board
					#if DEBUG
						imshow("Board", board);
					#endif

					markers.push_back(marker);
				}
			}

			return markers;
		}

		/**
		 * Get aruco marker bits data.
		 * @param image Square image with the aruco marker.
		 * @return Binary image with the aruco code  the image has resolution (cols + 2, rows + 2)
		 */
		static Mat processArucoImage(Mat image)
		{
			Mat aruco;
			resize(image, aruco, Size(7, 7));

			Mat gray;
			cvtColor(aruco, gray, CV_RGB2GRAY);

			Mat binary;
			threshold(gray, binary, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);;

			return binary;
		}

		/**
		 * Read aruco data from binary image with exactly the same size of the aruco marker.
		 * @param binary Binary image containing aruco info.
		 * @return ArucoMarker with the information collected.
		 */
		static ArucoMarker readArucoData(Mat binary)
		{
			ArucoMarker marker = ArucoMarker();

			for(unsigned int i = 0; i < binary.cols * binary.rows ; i++)
			{
				marker.cells[i / binary.cols][i % binary.cols] = (binary.data[i] == 255);
			}

			return marker;
		}

		/**
		 * Draw the border and id of all markers found on top of camera image.
		 * @param frame Image where to write Aruco information.
		 * @param markers Vector with all aruco markers found.
		 * @param camera Camera intrinsic calibration matrix.
		 * @param distortion Camera distortion calibration matrix.
		 */
		static void drawMarkers(Mat frame, vector<ArucoMarker> markers, Mat camera, Mat distortion)
		{
			for(unsigned int i = 0; i < markers.size(); i++)
			{
				Point2f center;

				//Draw countours
				for(unsigned int j = 0; j < 4; j++)
				{
					line(frame, markers[i].projected[j], markers[i].projected[(j + 1) % 4], Scalar(255, 0, 255), 2);
					center.x += markers[i].projected[j].x;
					center.y += markers[i].projected[j].y;
				}

				center.x /= 4;
				center.y /= 4;

				//Draw referencial
				Mat rotation, position;

				#if CV_MAJOR_VERSION == 2
					solvePnP(markers[i].info.world, markers[i].projected, camera, distortion, rotation, position, false, ITERATIVE);
				#else
					solvePnP(markers[i].info.world, markers[i].projected, camera, distortion, rotation, position, false, SOLVEPNP_ITERATIVE);
				#endif
				
				vector<Point3d> referencial;
				referencial.push_back(Point3d(0, 0, 0));
				referencial.push_back(Point3d(markers[i].info.size / 2, 0, 0));
				referencial.push_back(Point3d(0, markers[i].info.size / 2, 0));
				referencial.push_back(Point3d(0, 0, markers[i].info.size / 2));

				vector<Point2d> projected;
				projectPoints(referencial, rotation, position, camera, distortion, projected);

				line(frame, projected[0], projected[1], Scalar(0, 0, 255), 2);
				putText(frame, "X", projected[1], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

				line(frame, projected[0], projected[2], Scalar(0, 255, 0), 2);
				putText(frame, "Y", projected[2], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
			
				line(frame, projected[0], projected[3], Scalar(255, 0, 0), 2);
				putText(frame, "Z", projected[3], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);

				//Draw number
				putText(frame, to_string(markers[i].id), center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
			}
		}

		/**
		 * Draw origin of the referencial estimated from all markers found.
		 * @param frame Image where to write origin referencial.
		 * @param markers Vector with all aruco markers found.
		 * @param camera Camera intrinsic calibration matrix.
		 * @param distortion Camera distortion calibration matrix.
		 * @param size Size of the referencial.
		 */
		static void drawOrigin(Mat frame, vector<ArucoMarker> markers, Mat camera, Mat distortion, float size = 1)
		{
			if(markers.size() == 0)
			{
				return;
			}
			
			vector<Point3f> world;
			vector<Point2f> image;

			for(unsigned int i = 0; i < markers.size(); i++)
			{
				for(unsigned int k = 0; k < markers[i].projected.size(); k++)
				{
					world.push_back(markers[i].info.world[k]);
					image.push_back(markers[i].projected[k]);
				}

				//Draw countours
				for(unsigned int j = 0; j < 4; j++)
				{
					line(frame, markers[i].projected[j], markers[i].projected[(j + 1) % 4], Scalar(0, 150, 0), 2);
				}
			}

			Mat rotation, position;

			#if CV_MAJOR_VERSION == 2
				solvePnP(world, image, camera, distortion, rotation, position, false, ITERATIVE);
			#else
				solvePnP(world, image, camera, distortion, rotation, position, false, SOLVEPNP_ITERATIVE);
			#endif

			vector<Point3d> referencial;
			referencial.push_back(Point3d(0, 0, 0));
			referencial.push_back(Point3d(size, 0, 0));
			referencial.push_back(Point3d(0, size, 0));
			referencial.push_back(Point3d(0, 0, size));

			vector<Point2d> projected;
			projectPoints(referencial, rotation, position, camera, distortion, projected);

			line(frame, projected[0], projected[1], Scalar(0, 0, 255), 2);
			putText(frame, "X", projected[1], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

			line(frame, projected[0], projected[2], Scalar(0, 255, 0), 2);
			putText(frame, "Y", projected[2], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
			
			line(frame, projected[0], projected[3], Scalar(255, 0, 0), 2);
			putText(frame, "Z", projected[3], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
		}

		/**
		 * Preview all quads found in the image.
		 * Used for debug purpuse only.
		 * @param frame Original frame to extract color information
		 * @param quads Vector of quads detected in the frame.
		 * @return Image only with the found quads.
		 */
		static Mat previewQuads(Mat frame, vector<Quadrilateral> quads)
		{
			Mat sum = Mat::zeros(frame.rows, frame.cols, CV_8UC3);

			for(unsigned int i = 0; i < quads.size(); i++)
			{
				add(sum, filterQuadRegion(frame, quads[i]), sum);
			}

			return sum;
		}

		/**
		 * Draw aruco marker to binary image.
		 * @param marker Marker to be drawn.
		 * @param size Size of the output image.
		 * @return Image representing the marker.
		 */
		static Mat drawArucoMarker(ArucoMarker marker, Size size)
		{
			Mat out = Mat::zeros(7, 7, CV_8UC1);

			for(unsigned int i = 0; i < 7; i++)
			{
				for(unsigned int j = 0; j < 7; j++)
				{
					out.data[i * 7 + j] = marker.cells[i][j] * 255;
				}
			}

			resize(out, out, size, 0, 0, INTER_NEAREST);

			return out;
		}

		/**
		 * Crop image to quad area.
		 * Output size has the same size as the input.
		 * @param image Image to be cropped.
		 * @param quad Quad to be used to crop the image.
		 * @return Cropped image.
		 */
		static Mat filterQuadRegion(Mat image, Quadrilateral quad)
		{
			Mat out = Mat::zeros(image.rows, image.cols, CV_8UC3);

			Point p[1][4];
			p[0][0] = quad.points[0];
			p[0][1] = quad.points[1];
			p[0][2] = quad.points[2];
			p[0][3] = quad.points[3];

			const Point* points[1] = {p[0]};
			int points_count[] = {4};

			//Create Mask
			fillPoly(out, points, points_count, 1, Scalar(1, 1, 1));

			//Apply mask to image
			for(unsigned int i = 0; i < out.rows; i++)
			{
				for(unsigned int j = 0; j < out.cols; j++)
				{
					int t = (i*out.cols+j)*3;
					out.data[t] *= image.data[t];
					out.data[t+1] *= image.data[t+1];
					out.data[t+2] *= image.data[t+2];
				}
			}

			return out;
		}

		/**
		 * Apply inverse perspective transformation to image using quad.
		 * @param image Image to transform.
		 * @param size Size of the output image.
		 * @param quad Quad that defines the square area.
		 * @return Corrected image.
		 */
		static Mat deformQuad(Mat image, Point2i size, vector<Point2f> quad)
		{
			Mat out = Mat::zeros(size.x, size.y, CV_8UC3);

			vector<Point2f> points;
			points.push_back(Point2f(0, 0));
			points.push_back(Point2f(0, out.rows));
			points.push_back(Point2f(out.cols, out.rows));
			points.push_back(Point2f(out.cols, 0));

			Mat transformation = getPerspectiveTransform(quad, points);
			warpPerspective(image, out, transformation, out.size(), INTER_LINEAR);

			return out;
		}
};
