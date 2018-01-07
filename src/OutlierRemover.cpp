#include "OutlierRemover.h"
#include "math.h"

OutlierRemover::OutlierRemover()
{
}


OutlierRemover::~OutlierRemover()
{
}

void OutlierRemover::RemoveOutliers(PointCloud& pc)
{
	for (int i = 0; i < pc.rings.size(); i++)
	{
		for (int j = 0; j < pc.rings[i].GetPointCount(); j++)
		{
			//Remove points if between 81 and 95, -95 and -81, >125 angles. And if range <=2000
			float angle, range;
			pc.rings[i].GetPointRaw(j, angle, range);
			if (abs(angle) > 81 && abs(angle) < 95 || angle > 125 || range <= 2000)
			{
				pc.rings[i].SetPointValidity(j, false);
			}
		}
	}
}
