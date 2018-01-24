#include "OutlierRemover.h"
#include "math.h"
#include <iostream>

using namespace std;

OutlierRemover::OutlierRemover()
{
}


OutlierRemover::~OutlierRemover()
{
}

void OutlierRemover::RemoveOutliersBasedOnRaw(PointCloud& pc)
{
	for (int i = 0; i < pc.rings.size(); i++)
	{
		int pointCount = pc.rings[i].GetPointCount();
		for (int j = 0; j < pointCount; j++)
		{
			//Remove points if between 81 and 95, -95 and -81, >125 angles
			float angle, range;
			pc.rings[i].GetPointRaw(j, angle, range);
			if ((abs(angle) > 81 && abs(angle) < 95) || angle > 125 || range <= 1600 || range > 5800)
			{
				pc.rings[i].SetPointValidity(j, false);
			}
		}
	}

	for (int i = 1; i < pc.rings.size() - 1; i++)
	{
		int pointCount = pc.rings[i].GetPointCount();

		//If there's a big gap between a point and its neighbours, and everything's valid, make it invalid
		for (int j = 0; j < pointCount; j++)
		{
			//if (pc.rings[i].PointValid(j - 1) && pc.rings[i].PointValid(j) && pc.rings[i].PointValid(j + 1))
			{
				float angle, range, rangeBefore, rangeAfter;
				pc.rings[i].GetPointRaw(j, angle, range);
				pc.rings[i - 1].GetPointRaw(j, angle, rangeBefore);
				pc.rings[i + 1].GetPointRaw(j, angle, rangeAfter);

				if (abs(range - rangeBefore) > 100 && abs(range - rangeAfter) > 100)
				{
					pc.rings[i].SetPointValidity(j, false);
				}
			}
		}
	}
}

//todo: may want to multi-thread
void OutlierRemover::RemoveOutliersBasedOnLines(PointCloud& pc, LineFitter& lf)
{
	for (int i = 0; i < pc.rings.size(); i++)
	{
		int pointCount = pc.rings[i].GetPointCount();
		for (int j = 0; j < pointCount; j++)
		{
			if (pc.rings[i].PointValid(j))
			{
				auto point2D = pc.rings[i].GetPointAligned(j);
				auto point3D = Eigen::Vector3d(point2D(0), point2D(1), i);
				float distance = lf.CalculateDistance(point3D);
				if (distance > 200)
				{
					pc.rings[i].SetPointValidity(j, false);
				}
			}
		}
		cout << "Ring " << i << " validated against line" << endl;
	}
}
