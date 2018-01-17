#pragma once
#include "PointCloud.h"

class ScanMatcher
{
public:
	ScanMatcher();
    void DoRANSAC(PointCloud& pc);

private:
    Eigen::Vector2d DoICP(PointCloud& pc, int ringStart, std::vector<int> pointIDs);
    Eigen::Vector2d DoAlignment(PointCloud& pc, int ringStart, std::vector<int> pointIDs);
    Eigen::Vector2d DoAlignmentSum(PointCloud& pc, int ringStart, std::vector<int> pointIDs);
	int iterations = 500;
    int numTestPoints = 100;
};