#include "PointCloud.h"
#include <fstream>
#include <iostream>
#include <boost/tokenizer.hpp>

using namespace std;

PointCloud::PointCloud()
{
}


PointCloud::~PointCloud()
{
}

void PointCloud::Load(std::string path)
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

			if (rings.size() > 100)	//TODO: REMOVE
			{
				break;
			}
			ring = Ring();
		}
	}

	file.close();
}

