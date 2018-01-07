#include "Ring.h"
#include <boost/math/constants/constants.hpp>

using namespace std;

Ring::Ring()
{
	points.reserve(1080);
}


Ring::~Ring()
{
}

int Ring::AddPoint(float range, float angle)
{
	angle = angle * boost::math::constants::pi<double>() / 180.0;
	Point pt;
	pt.pos = Eigen::Vector2d(sin(angle) * range, cos(angle) * range);
	pt.range = range;
	pt.angle = angle;
	pt.valid = true;
	points.push_back(pt);
	return pt.valid ? 1 : 0;
}

Eigen::Vector2d Ring::GetPointAligned(int id)
{
	return points[id].pos + moveToBeAligned;
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
