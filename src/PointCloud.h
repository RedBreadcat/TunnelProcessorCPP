#pragma once
#include <vector>
#include <string>
#include "Ring.h"

class PointCloud
{
public:
	PointCloud();
	~PointCloud();
	void Load(std::string path);
	void SaveAdjustments(std::string path);
	std::vector<Ring> rings;
};

