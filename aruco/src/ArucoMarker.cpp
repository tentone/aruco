#pragma once

#include <string>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>

#include "ArucoMarkerInfo.cpp"

using namespace cv;
using namespace std;

/**
 * Class is used to represent aruco markers.
 * All markers are assumed to be 5x5 markers capable of 1024 options, after considering rotation and error detection.
 */
class ArucoMarker
{
	public:
		/**
		 * Contain all the cells in the marker.
		 * Cells are encoded as [x][y] from left to right on X and from up to down on Y.
		 */
		int cells[7][7];

		/**
		 * Number of rows used to store data in the marker.
		 */
		int rows;

		/**
		 * Number of cols used to store data in the marker.
		 */
		int cols;

		/**
		 * Number of 90 degrees turns applied to the marker.
		 */
		int rotation;

		/**
		 * Id of this markers.
		 * Can be calculated after filling up the cells info.
		 */
		int id;

		/**
		 * Flag to store if the marker was already validated.
		 */
		int validated;

		/**
		 * Store info about the marker real world morphology and location.
		 */
		ArucoMarkerInfo info;

		/**
		 * Projected corner points in camera coordinates.
		 * This vector should have only 4 points.
		 * But can be used to store extra known points for the marker.
		 */
		vector<Point2f> projected;

		/**
		 * Aruco marker constructor.
		 */
		ArucoMarker()
		{
			rows = 5;
			cols = 5;
			id = -1;
			rotation = 0;
			validated = false;
		}

		/**
		 * Attach info to this marker.
		 *
		 * @param info Marker information.
		 */
		void attachInfo(ArucoMarkerInfo _info)
		{
			info = _info;
		}

		/**
		 * Get the id of the marker. The ID its a value between 0 and 1024.
		 *
		 * @return ID of this aruco marker.
		 */
		int calculateID()
		{
			id = 0;

			for(int i = 1; i < 6; ++i)
			{
				id <<= 1;
				id |= cells[i][2];
				id <<= 1;
				id |= cells[i][4];
			}

			return id;
		}

		/**
		 * Calculate all parameters and check if its a valid aruco marker.
		 * Should be called only after projected points and cell info is added.
		 * @return true if the marker is valid, false otherwise.
		 */
		bool validate()
		{
			validated = false;

			//Check if there are points
			if(projected.size() == 0)
			{
				return false;
			}

			//Check black border allow up to three white squares for edge light bleed cases
			unsigned int bad = 0;
			for(unsigned int i = 0; i < 7; i++)
			{
				if(cells[i][0] != 0 || cells[i][6] != 0 || cells[0][i] != 0 || cells[6][i] != 0)
				{
					bad++;
					if(bad > 3)
					{
						return false;
					}
				}
			}

			//Check hamming distance of internal data
			for(int j = 0; j < 4; j++)
			{
				if(hammingDistance() == 0)
				{
					calculateID();
					validated = true;
					return true;
				}

				rotate();
			}

			return false;
		}

		/**
		 * Used to rotate marker 90 degrees.
		 * Can be used to rotate the marker data and make sure that it is read properly.
		 */
		void rotate()
		{
			int temp[7][7];
			int n = 7;

			for (int i = 0; i < n; i++)
			{
				for (int j = 0; j < n; j++)
				{
					temp[i][j] = cells[n - j - 1][i];
				}
			}

			for (int i = 0; i < n; i++)
			{
				for (int j = 0; j < n; j++)
				{
					cells[i][j] = temp[i][j];
				}
			}

			rotation++;

			std::rotate(projected.begin(), projected.begin() + 1, projected.end());
		}

		/**
		 * Calculates the sum of the hamming distance (number of diferent bits) for this marker relative to the ids matrix used to validate the aruco markers.
		 */
		int hammingDistance()
		{
			int ids[4][5] = {
				{1, 0, 0, 0, 0},
				{1, 0, 1, 1, 1},
				{0, 1, 0, 0, 1},
				{0, 1, 1, 1, 0}
			};

			int dist = 0;
			int sum, minSum;

			for(int i = 1; i < 6; ++i)
			{
				minSum = 99999;

				for(int j = 1; j < 5; ++j)
				{
					sum = 0;

					for(int k = 1; k < 6; ++k)
					{
						sum += cells[i][k] == ids[j - 1][k - 1] ? 0 : 1;
					}

					if(sum < minSum)
					{
						minSum = sum;
					}
				}

				dist += minSum;
			}

			return dist;
		}

		/**
		 * Print marker cells to the stdout.
		 */
		void print()
		{
			cout << "{" << endl;
			cout << "    Valid: " << validated << endl;
			cout << "    Hamming: " << hammingDistance() << endl;
			cout << "    ID: " << id << endl;
			cout << "    Cells: [";
			for(int i = 0; i < 7; i++)
			{
				for(int j = 0; j < 7; j++)
				{
					cout << cells[i][j] << ", ";
				}

				if(i == 6)
				{
					cout << "]" << endl;
				}
				else
				{
					cout << endl << "            ";
				}
			}
			cout << "    Rotation: " << rotation << endl;

			for(unsigned int i = 0; i < projected.size(); i++)
			{
				cout << "    Projected: " << projected[i].x << ", " << projected[i].y << endl;
			}

			info.print();

			cout << "}" << endl;
		}
};
