#pragma once
#include "PointCloud.h"

class ScanMatcher
{
public:
	ScanMatcher();
    void DoRANSAC(PointCloud& pc);

private:
    void DoICP(PointCloud& pc, int ringStart, std::vector<int> pointIDs);
	int iterations = 5;
    int numTestPoints = 10;
};