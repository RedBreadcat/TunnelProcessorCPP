#include "Ring.h"
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

Eigen::Vector2d Ring::GetPointAligned(int id) const
{
	return points[id].pos + moveToBeAligned;
}

Eigen::Vector2d Ring::GetPointRotated(int id) const
{
	float s = sin(angle);
	float c = cos(angle);

	auto pt = GetPointAligned(id);
	double xNew = pt[0] * c - pt[1] * s;
	double yNew = pt[0] * s + pt[1] * c;
	return Eigen::Vector2d(xNew, yNew);
}

int Ring::GetClosestPoint(int pointID, const Ring& compareRing, float& shortestDistance)
{
	int startID = pointID - 10;	//Only going to look for the closest point within a small point ID range
	if (startID < 0)
	{
		startID = 0;
	}
	int endID = pointID + 10;
	if (endID > points.size())
	{
		endID = points.size();
	}

	shortestDistance = HUGE_VALF;
	int closestPointID = -1;
	Eigen::Vector2d refPoint = GetPointAligned(pointID);
	for (int p = startID; p < endID; p++)
	{
		Eigen::Vector2d otherPoint = compareRing.GetPointAligned(p);

		double distance = sqrt((refPoint[0] - otherPoint[0])*(refPoint[0] - otherPoint[0]) + (refPoint[1] - otherPoint[1])*(refPoint[1] - otherPoint[1]));

		if (distance < shortestDistance)
		{
			shortestDistance = distance;
			closestPointID = p;
		}
	}

	if (closestPointID == -1)
	{
		return pointID;
	}
	else
	{
		return closestPointID;
	}
}

int Ring::GetClosestPointRot(int pointID, const Ring& compareRing, float& shortestDistance)
{
	int startID = pointID - 10;	//Only going to look for the closest point within a small point ID range
	if (startID < 0)
	{
		startID = 0;
	}
	int endID = pointID + 10;
	if (endID > points.size())
	{
		endID = points.size();
	}

	shortestDistance = HUGE_VALF;
	int closestPointID = -1;
	Eigen::Vector2d refPoint = GetPointRotated(pointID);
	for (int p = startID; p < endID; p++)
	{
		Eigen::Vector2d otherPoint = compareRing.GetPointRotated(p);

		double distance = sqrt((refPoint[0] - otherPoint[0])*(refPoint[0] - otherPoint[0]) + (refPoint[1] - otherPoint[1])*(refPoint[1] - otherPoint[1]));

		if (distance < shortestDistance)
		{
			shortestDistance = distance;
			closestPointID = p;
		}
	}

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
