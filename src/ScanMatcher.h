#pragma once
#include "PointCloud.h"

class ScanMatcher
{
public:
	ScanMatcher();
    void DoRANSAC(PointCloud& pc);

private:
	void DoAlignment(PointCloud& pc, int ringStart, const std::vector<int>& pointIDs, Eigen::Vector2d& t, float& r);
	float CalcError(PointCloud& pc, int ringStart, int pointID);
	void TestAlignment(PointCloud& pc);
	int iterations = 200;
    int numTestPoints = 200;
};