#include <iostream>
#include "PointCloud.h"
#include "OutlierRemover.h"
#include "LineFitter.h"
#include "ScanMatcher.h"

using namespace std;

int main()
{
	PointCloud pc;

	pc.Load("/home/roby/tunneldata/yarraValley_scan_1.txt");
	OutlierRemover::RemoveOutliers(pc);

	ScanMatcher sm;
	sm.DoRANSAC(pc);

	//LineFitter lf;
	//lf.Fit3DLine(pc);

	//auto testPoint = Eigen::Vector3d(1000, 0, 0);
	//cout << lf.CalculateDistance(testPoint) << endl;

	return 0;
}