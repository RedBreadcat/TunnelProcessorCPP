#pragma once
#include "PointCloud.h"
#include "LineFitter.h"

class OutlierRemover
{
public:
	OutlierRemover();
	~OutlierRemover();

	static void RemoveOutliersBasedOnRaw(PointCloud& pc);
	static void RemoveOutliersBasedOnLines(PointCloud& pc, LineFitter& lf);
};

