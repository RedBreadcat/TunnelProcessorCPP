#pragma once
#include "PointCloud.h"

class OutlierRemover
{
public:
	OutlierRemover();
	~OutlierRemover();

	static void RemoveOutliers(PointCloud& pc);
};

