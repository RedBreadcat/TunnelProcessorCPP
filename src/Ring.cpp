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
