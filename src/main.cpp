#include <iostream>
#include "PointCloud.h"
#include "OutlierRemover.h"
#include "LineFitter.h"
#include "ScanMatcher.h"

using namespace std;

int main()
{
	string pathWithoutExtension = "/home/roby/tunneldata/yarraValley_scan_1";
	PointCloud pc;

	pc.Load(pathWithoutExtension + ".txt");
	OutlierRemover::RemoveOutliers(pc);

	ScanMatcher sm;
	sm.DoRANSAC(pc);

	LineFitter lf;
	lf.Fit3DLine(pc);

	pc.SaveAdjustments(pathWithoutExtension);
	lf.SaveLines(pathWithoutExtension);

	//auto testPoint = Eigen::Vector3d(1000, 0, 0);
	//cout << lf.CalculateDistance(testPoint) << endl;

	return 0;
}