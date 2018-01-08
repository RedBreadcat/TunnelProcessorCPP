#pragma once
#include <vector>
#include "PointCloud.h"
#include "Line.h"

class LineFitter
{
public:
	LineFitter();
	void Fit3DLine(PointCloud& pc);
	double CalculateDistance(Eigen::Vector3d point3D);
	void SaveLines(std::string path);

private:
	bool FitPoints(const std::vector<double> &yAxis, const std::vector<double> &tList, double& slope, double& intercept);
	std::vector<Line> lines;
};

