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
	
	//When drone rotates, it captures the floor on edges of the scan. This is bad data which we can ignore by setting these
	static int pointStartAdd;
	static int pointEndSub;
};

