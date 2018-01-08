#include "PointCloud.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <boost/tokenizer.hpp>

using namespace std;

PointCloud::PointCloud()
{
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
		for (int j = 0; j < rings[i].GetPointCount(); i++)
		{
			//TODO: Write whether point is valid or not. A bool? The ID? All point IDs, followed by a bool?
			//Do whatever will give good performance on visualiser side.
		}
		cout << "***" << endl;
		adjustmentsFile << setprecision(10) << rings[i].moveToBeAligned[0] << endl;
		adjustmentsFile << setprecision(10) << rings[i].moveToBeAligned[1] << endl;
	}
	adjustmentsFile.close();
}
