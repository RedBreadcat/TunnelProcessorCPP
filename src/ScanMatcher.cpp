#include "ScanMatcher.h"
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <random>

using namespace std;

ScanMatcher::ScanMatcher()
{
}

//https://en.wikipedia.org/wiki/Random_sample_consensus
void ScanMatcher::DoRANSAC(PointCloud& pc)
{
    random_device rd;
    mt19937 rng(rd());
    uniform_int_distribution<int> uniformIntegers(PointCloud::pointStartAdd, 1079 - PointCloud::pointEndSub);

	for (int i = 0; i < pc.rings.size() - 1; i++)
	{
		int it = 0;
		double bestError = HUGE_VALF;
		double lastBestError = HUGE_VALF;
		auto bestTranslation = Eigen::Vector2d(0, 0);
		float bestRotation = 0;
		while (it++ < iterations)
		{
			//Select random points
			vector<int> possibleInliersForTesting;
			possibleInliersForTesting.reserve(numTestPoints);
			while (possibleInliersForTesting.size() < numTestPoints)
			{
				int pointID = uniformIntegers(rng);
				//If point in this and next ring are valid, and point hasn't been added before
				//Find statement is equivalent to "does not contain point"
				if (pc.rings[i].PointValid(pointID) && pc.rings[i + 1].PointValid(pointID) && find(possibleInliersForTesting.begin(), possibleInliersForTesting.end(), pointID) == possibleInliersForTesting.end())
				{
					possibleInliersForTesting.push_back(pointID);
				}
			}

			Eigen::Vector2d translation;
			float rotation;
			DoAlignment(pc, i, possibleInliersForTesting, translation, rotation);
			pc.rings[i + 1].translation = translation;
			pc.rings[i + 1].rotation = rotation;

			//Calculate error of the model against all points
			int pointCount = pc.rings[i].GetPointCount() - PointCloud::pointEndSub;
			double accumulatedError = 0;
			for (int j = PointCloud::pointStartAdd; j < pointCount; j++)
			{
				accumulatedError += CalcError(pc, i, j);
			}

			double averageError = accumulatedError / possibleInliersForTesting.size();
			if (averageError < bestError)
			{
				bestTranslation = translation;
				bestRotation = rotation;
				lastBestError = bestError;
				bestError = averageError;
			}
		}

		pc.rings[i + 1].translation = bestTranslation;
		pc.rings[i + 1].rotation = bestRotation;
		cout << "Ring: " << i << endl;
		cout << "Best translation: " << bestTranslation(0) << ", " << bestTranslation(1) << endl;
		cout << "Best rotation: " << bestRotation * 180 / boost::math::constants::pi<float>() << " degrees" << endl;
		cout << "Best error: " << bestError << endl;
		cout << "Delta: " << lastBestError - bestError << endl;
		cout << endl;
	}
}

//http://nghiaho.com/?page_id=671
void ScanMatcher::DoAlignment(PointCloud& pc, int ringStart, const vector<int>& pointIDs, Eigen::Vector2d& t, float& r) //RefReturns the amount that each ring+1 point needs to move to be aligned
{
	auto centroidNext = Eigen::Vector2d(0, 0);
	auto centroidRef = Eigen::Vector2d(0, 0);
	for (int i = 0; i < pointIDs.size(); i++)
	{
		centroidRef += pc.rings[ringStart].GetPointAligned(pointIDs[i]);	//Remember that current ring must be aligned
		centroidNext += pc.rings[ringStart + 1].GetPoint(pointIDs[i]);
	}
	centroidRef /= (float)pointIDs.size();
	centroidNext /= (float)pointIDs.size();

	Eigen::Matrix2d h;
	for (int i = 0; i < pointIDs.size(); i++)
	{
		Eigen::Matrix<double, 1, 2> ptRefT = (pc.rings[ringStart].GetPointAligned(pointIDs[i]) - centroidRef).transpose();	//Aligned

		h += (pc.rings[ringStart + 1].GetPoint(pointIDs[i]) - centroidNext) * ptRefT;
	}

	Eigen::JacobiSVD<Eigen::Matrix2d> usv = h.jacobiSvd(Eigen::DecompositionOptions::ComputeThinU | Eigen::DecompositionOptions::ComputeThinV);	//U and V aren't calculated by default

	auto rotationM = usv.matrixV() * usv.matrixU().transpose();	//Produces a 2D rotation matrix
	r = -acos(rotationM(0, 0));	//Extract rotation from matrix
	t = -rotationM * centroidNext + centroidRef;
}

float ScanMatcher::CalcError(PointCloud& pc, int ringStart, int pointID)
{
	float error;
	pc.rings[ringStart].GetClosestPoint(pointID, pc.rings[ringStart + 1], error);
	return error;
}

void ScanMatcher::TestAlignment(PointCloud& pc)
{
	pc.rings[0].points[0].pos = Eigen::Vector2d(0, 1);
	pc.rings[0].points[1].pos = Eigen::Vector2d(0, -1);
	pc.rings[0].points[2].pos = Eigen::Vector2d(1, 1);

	pc.rings[1].points[0].pos = Eigen::Vector2d(0, 1) + Eigen::Vector2d(1, 1);
	pc.rings[1].points[1].pos = Eigen::Vector2d(0, -1) + Eigen::Vector2d(1, 1);
	pc.rings[1].points[2].pos = Eigen::Vector2d(1, 1) + Eigen::Vector2d(1, 1);

	vector<int> points;
	points.push_back(0);
	points.push_back(1);
	points.push_back(2);

	Eigen::Vector2d t;
	float r;
	DoAlignment(pc, 0, points, t, r);
	cout << t(0) << ", " << t(1) << endl;
	cout << r << endl;
}