#pragma once
#include <vector>
#include "Point.h"

class Ring
{
public:
	Ring();
	~Ring();
	int AddPoint(float range, float angle);
	Eigen::Vector2d GetPointAligned(int id);
	void GetPointRaw(int id, float& angle, float& range);
	int GetPointCount();
	bool PointValid(int id);
	void SetPointValidity(int id, bool valid);

	Eigen::Vector2d moveToBeAligned;

private:
	std::vector<Point> points;
};
