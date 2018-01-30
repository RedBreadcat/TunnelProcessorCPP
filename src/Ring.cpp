#include "Ring.h"
#include "PointCloud.h"
#include <boost/math/constants/constants.hpp>
#include <iostream>

using namespace std;

Ring::Ring()
{
	points.reserve(1080);
}


Ring::~Ring()
{
}

void Ring::AddPoint(float range, float angle)
{
	angle = angle * boost::math::constants::pi<double>() / 180.0;
	Point pt;
	pt.pos = Eigen::Vector2d(sin(angle) * range, cos(angle) * range);
	pt.range = range;
	pt.angle = angle;
	pt.valid = true;
	pt.pickedForRANSAC = false;
	points.push_back(pt);
}

Eigen::Vector2d Ring::GetPoint(int id) const
{
	return points[id].pos;
}

Eigen::Vector2d Ring::GetPointAligned(int id) const
{
	float s = sin(rotation);
	float c = cos(rotation);

	Eigen::Vector2d aligned = Eigen::Vector2d(points[id].pos(0) * c - points[id].pos(1) * s, points[id].pos(0) * s + points[id].pos(1) * c);
	aligned += translation;

	return aligned;
}

int Ring::GetClosestPoint(int pointID, const Ring& compareRing, float& shortestDistance)
{
	int startID = pointID - 50;	//Only going to look for the closest point within a small point ID range
	if (startID < PointCloud::pointStartAdd)
	{
		startID = PointCloud::pointStartAdd;
	}
	int endID = pointID + 50;
	if (endID > points.size() - PointCloud::pointEndSub)
	{
		endID = points.size() - PointCloud::pointEndSub;
	}

	shortestDistance = HUGE_VALF;
	int closestPointID = -1;
	Eigen::Vector2d refPoint = GetPointAligned(pointID);
	for (int p = startID; p < endID; p++)
	{
		Eigen::Vector2d otherPoint = compareRing.GetPointAligned(p);

		double distance = (refPoint[0] - otherPoint[0])*(refPoint[0] - otherPoint[0]) + (refPoint[1] - otherPoint[1])*(refPoint[1] - otherPoint[1]);

		if (distance < shortestDistance)
		{
			shortestDistance = distance;
			closestPointID = p;
		}
	}

	shortestDistance = sqrt(shortestDistance);

	if (closestPointID == -1)
	{
		return pointID;
	}
	else
	{
		return closestPointID;
	}
}

void Ring::GetPointRaw(int id, float& angle, float& range)
{
	angle = points[id].angle;
	range = points[id].range;
}

int Ring::GetPointCount()
{
	return points.size();
}

bool Ring::PointValid(int id)
{
	return points[id].valid;
}

void Ring::SetPointValidity(int id, bool valid)
{
	points[id].valid = valid;
}
