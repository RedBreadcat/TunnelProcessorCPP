#pragma once
#include "PointCloud.h"

class ScanMatcher
{
public:
	ScanMatcher();
    void DoRANSAC(PointCloud& pc);

private:
    Eigen::Vector2d DoAlignmentCentroid(PointCloud& pc, int ringStart, const std::vector<int>& pointIDs);
	float CalcError(PointCloud& pc, int ringStart, int pointID);
	float CalcErrorRot(PointCloud& pc, int ringStart, int pointID);
	int iterations = 200;
    int numTestPoints = 200;
};