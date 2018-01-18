#include <iostream>
#include "PointCloud.h"
#include "OutlierRemover.h"
#include "LineFitter.h"
#include "ScanMatcher.h"

using namespace std;

int main()
{
	string pathWithoutExtension = "C:\\Users\\Roby\\OneDrive\\Documents\\Tunnel\\Data\\yarraValley_scan_1";
	PointCloud pc;

	pc.Load(pathWithoutExtension + ".txt");
	OutlierRemover::RemoveOutliersBasedOnRaw(pc);

	ScanMatcher sm;
	sm.DoRANSAC(pc);

	LineFitter lf;
	lf.Fit3DLine(pc);

	//Note: is too strict
	//OutlierRemover::RemoveOutliersBasedOnLines(pc, lf);

	pc.SaveAdjustments(pathWithoutExtension);
	lf.SaveLines(pathWithoutExtension);

	getchar();
	return 0;
}