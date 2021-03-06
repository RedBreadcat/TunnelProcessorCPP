#include "PointCloud.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <boost/tokenizer.hpp>

using namespace std;

//Static varables must be defined
int PointCloud::pointStartAdd;
int PointCloud::pointEndSub;

PointCloud::PointCloud()
{
	pointStartAdd = 50;
	pointEndSub = 50;
}


PointCloud::~PointCloud()
{
}

void PointCloud::Load(string path)
{
	ifstream file(path);
	string line;

	Ring ring;
	int i = 0;
	int x = 0;

	boost::char_separator<char> sep(",");
	while (getline(file, line))
	{
		boost::tokenizer<boost::char_separator<char>> tokens(line, sep);

		auto tokenIter = tokens.begin();
		int id = stoi(*tokenIter++);
		float angle = stod(*tokenIter++);	//Not necessary given that angle is same each time. Steps aren't linear though, so I at least need to think
		float range = stod(*tokenIter);

		ring.AddPoint(range, angle);

		if (id == 1080)
		{
			rings.push_back(ring);
			ring = Ring();
		}
	}

	file.close();
}

void PointCloud::SaveAdjustments(string path)
{
	ofstream adjustmentsFile(path + "_adjustments.txt");

	for (int i = 0; i < rings.size(); i++)
	{
		adjustmentsFile << "RING" << endl;
		adjustmentsFile << setprecision(10) << rings[i].translation[0] << endl;
		adjustmentsFile << setprecision(10) << rings[i].translation[1] << endl;
		adjustmentsFile << setprecision(10) << rings[i].rotation << endl;
		int pointCount = rings[i].GetPointCount();
		for (int j = 0; j < pointCount; j++)	//Write the IDs of all the invalid points
		{
			if (!rings[i].PointValid(j))
			{
				adjustmentsFile << j << endl;
			}
		}
		adjustmentsFile << "RANSAC" << endl;
		for (int j = 0; j < pointCount; j++)
		{
			if (rings[i].points[j].pickedForRANSAC)
			{
				adjustmentsFile << j << endl;
			}
		}
	}
	adjustmentsFile.close();
	cout << "Saved adjustments file" << endl;
}
