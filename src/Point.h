#pragma once
#include <Eigen/Dense>

struct Point
{
	bool valid;
	Eigen::Vector2d pos;
	float range;
	float angle;
};