#include <iostream>
#include "PointCloud.h"
#include "OutlierRemover.h"
#include "LineFitter.h"

using namespace std;

int main()
{
	PointCloud pc;

	pc.Load("C:\\Users\\Roby\\OneDrive\\Documents\\Tunnel\\Data\\yarraValley_scan_1.txt");
	OutlierRemover::RemoveOutliers(pc);

	LineFitter lf;
	lf.Fit3DLine(pc);

	auto testPoint = Eigen::Vector3d(1000, 0, 0);
	cout << lf.CalculateDistance(testPoint) << endl;

	getchar();
	getchar();
	return 0;
}