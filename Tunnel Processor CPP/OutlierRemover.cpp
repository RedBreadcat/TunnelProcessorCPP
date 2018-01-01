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
		for (int j = 0; j < pc.rings[i].points.size(); j++)
		{
			//Remove points if between 81 and 95, -95 and -81, >125 angles. And if range <=2000
			if (abs(pc.rings[i].points[j].angle) > 81 && abs(pc.rings[i].points[j].angle) < 95 || pc.rings[i].points[j].angle > 125 || pc.rings[i].points[j].range <= 2000)
			{
				pc.rings[i].points[j].valid = false;
			}
		}
	}
}
