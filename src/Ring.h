#pragma once
#include <vector>
#include "Point.h"

class Ring
{
public:
	Ring();
	~Ring();
	int AddPoint(float range, float angle);

	std::vector<Point> points;
};

