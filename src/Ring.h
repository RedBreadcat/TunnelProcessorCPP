#pragma once
#include <vector>
#include "Point.h"

class Ring
{
public:
	Ring();
	~Ring();
	void AddPoint(float range, float angle);
	Eigen::Vector2d GetPointAligned(int id) const;
	Eigen::Vector2d GetPointRotated(int id) const;
	int GetClosestPoint(int pointID, const Ring& compareRing, float& shortestDistance);
	int GetClosestPointRot(int pointID, const Ring& compareRing, float& shortestDistance);
	void GetPointRaw(int id, float& angle, float& range);
	int GetPointCount();
	bool PointValid(int id);
	void SetPointValidity(int id, bool valid);

	Eigen::Vector2d moveToBeAligned = Eigen::Vector2d(0, 0);
	double angle = 0;	//Radians
	std::vector<Point> points;
};

