#include "LineFitter.h"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

LineFitter::LineFitter()
{
}

/* Paramterise line into 3 equations:
* x = x0 + at
* y = y0 + bt
* z = z0 + ct
* Where t is the ring number
*/
void LineFitter::Fit3DLine(PointCloud& pc)
{
	for (int i = 0; i < 1080; i++)
	{
		vector<double> xList;
		vector<double> yList;
		for (int j = 0; j < pc.rings.size(); j++)
		{
			if (pc.rings[j].PointValid(i))
			{
				Eigen::Vector2d pt = pc.rings[j].GetPointAligned(i);
				xList.push_back(pt[0]);
				yList.push_back(pt[1]);
				//todo: z
			}
		}

		if (xList.size() > pc.rings.size() * 0.75f)   //If at least 75% of the points are valid
		{
			vector<double> tList, zList;
			tList.reserve(xList.size());
			zList.reserve(xList.size());

			for (int w = 0; w < xList.size(); w++)   //Todo: The z/t values need to keep their value at an index. So they don't count from 1 to the end. Can make the full list at the start, then remove elements
			{
				tList.push_back(w);
				zList.push_back(w);
			}

			double xSlope, xIntercept;
			if (FitPoints(xList, tList, xSlope, xIntercept))
			{
				double ySlope, yIntercept;
				if (FitPoints(yList, tList, ySlope, yIntercept))
				{
					double zSlope, zIntercept;
					if (FitPoints(zList, tList, zSlope, zIntercept))
					{
						Line l(xIntercept, xSlope, yIntercept, ySlope, zIntercept, zSlope);
						lines.push_back(l);
					}
				}
			}
		}
		else
		{
			cout << "Not enough valid points for line " << i << ". Only " << xList.size() << " valid points" << endl;
		}
	}
}

bool LineFitter::FitPoints(const std::vector<double> &yAxis, const std::vector<double> &tList, double& slope, double& intercept)
{
	int nPoints = yAxis.size();
	double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
	for (int i = 0; i < nPoints; i++)
	{
		sumX += tList[i];
		sumY += yAxis[i];
		sumXY += tList[i] * yAxis[i];
		sumX2 += tList[i] * tList[i];
	}

	double xMean = sumX / nPoints;
	double yMean = sumY / nPoints;
	double denominator = sumX2 - sumX * xMean;
	// You can tune the eps (1e-7) below for your specific task
	if (std::fabs(denominator) < 1e-7) {
		cout << "Warning: line vertical" << endl;
		
		return false;
	}
	slope = (sumXY - sumX * yMean) / denominator;
	intercept = yMean - slope * xMean;
	return true;
}

double LineFitter::CalculateDistance(Eigen::Vector3d point3D)
{
	double shortestDistance = HUGE_VALF;
	for (int i = 0; i < lines.size(); i++)
	{
		double distance = lines[i].CalculateDistanceFast(point3D);

		if (distance < shortestDistance)
		{
			shortestDistance = distance;
		}
	}

	return shortestDistance;
}

void LineFitter::SaveLines(string path)
{
	ofstream linesFile(path + "_lines.txt");

	for (int i = 0; i < lines.size(); i++)
	{
		linesFile << setprecision(10) << lines[i].xLine[0] << endl;	//Intercept
		linesFile << setprecision(10) << lines[i].xLine[1] << endl;	//t coefficient
		linesFile << setprecision(10) << lines[i].yLine[0] << endl;	//Intercept
		linesFile << setprecision(10) << lines[i].yLine[1] << endl;	//t coefficient
		linesFile << setprecision(10) << lines[i].zLine[0] << endl;	//Intercept
		linesFile << setprecision(10) << lines[i].zLine[1] << endl;	//t coefficient
	}
	linesFile.close();
	cout << "Saved lines file" << endl;
}