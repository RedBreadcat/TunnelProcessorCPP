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
    uniform_int_distribution<int> uniformIntegers(0, 1079);	//1079

	for (int i = 0; i < pc.rings.size() - 1; i++)
	{
		int it = 0;
		double bestError = HUGE_VALF;
		double lastBestError = HUGE_VALF;
		auto bestAdjustment = Eigen::Vector2d(0, 0);
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

			Eigen::Vector2d adjustment = DoAlignmentCentroid(pc, i, possibleInliersForTesting);
			pc.rings[i + 1].moveToBeAligned = adjustment;

			//Calculate error of the model against all points
			int pointCount = pc.rings[i].GetPointCount();
			double accumulatedError = 0;
			for (int j = 0; j < pointCount; j++)
			{
				accumulatedError += CalcError(pc, i, j);
			}

			double averageError = accumulatedError / possibleInliersForTesting.size();
			if (averageError < bestError)
			{
				bestAdjustment = adjustment;
				lastBestError = bestError;
				bestError = averageError;
			}
		}

		pc.rings[i + 1].moveToBeAligned = bestAdjustment;
		cout << "Ring: " << i << endl;
		cout << "Best adjustment: " << bestAdjustment(0) << ", " << bestAdjustment(1) << endl;
		cout << "Best error: " << bestError << endl;
		cout << "Delta: " << lastBestError - bestError << endl;
		cout << endl;
	}

	/*
	//Rotation
	uniform_real_distribution<float> uniformFloats(-boost::math::constants::pi<float>() / 32.0f, boost::math::constants::pi<float>() / 32.0f);
	for (int i = 0; i < pc.rings.size() - 1; i++)
	{
		int it = 0;
		double bestError = HUGE_VALF;
		double lastBestError = HUGE_VALF;
		float bestAngle = 0;
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

			float angle = it == 1 ? pc.rings[i].angle : uniformFloats(rng);	//If on first iteration (usually it==0 but we've already increased it by this point), then compare to previous rotation
			pc.rings[i + 1].angle = angle;

			//Calculate error of the model against all points
			int pointCount = pc.rings[i].GetPointCount();
			double accumulatedError = 0;
			for (int j = 0; j < pointCount; j++)
			{
				accumulatedError += CalcErrorRot(pc, i, j);
			}

			double averageError = accumulatedError / possibleInliersForTesting.size();
			if (averageError < bestError)
			{
				bestAngle = angle;
				lastBestError = bestError;
				bestError = averageError;
			}
		}

		pc.rings[i + 1].angle = bestAngle;
		cout << "Ring: " << i << endl;
		cout << "Best angle: " << bestAngle << endl;
		cout << "Best error: " << bestError << endl;
		cout << "Delta: " << lastBestError - bestError << endl;
		cout << endl;
	}*/
}

Eigen::Vector2d ScanMatcher::DoAlignmentCentroid(PointCloud& pc, int ringStart, const vector<int>& pointIDs) //Returns the amount that each ring+1 point needs to move to be aligned
{
    auto centroidRef = Eigen::Vector2d(0, 0);
    auto centroidNext = Eigen::Vector2d(0, 0);
	for (int i = 0; i < pointIDs.size(); i++)
    {
        int pointID = pointIDs[i];
        centroidRef += pc.rings[ringStart].GetPointAligned(pointID);
        centroidNext += pc.rings[ringStart + 1].GetPointAligned(pointID);
    }
	centroidRef /= (float)pointIDs.size();
	centroidNext /= (float)pointIDs.size();
	return centroidRef - centroidNext;
}

float ScanMatcher::CalcError(PointCloud& pc, int ringStart, int pointID)
{
	float error;
	pc.rings[ringStart].GetClosestPoint(pointID, pc.rings[ringStart + 1], error);
	return error;
}

float ScanMatcher::CalcErrorRot(PointCloud& pc, int ringStart, int pointID)
{
	float error;
	pc.rings[ringStart].GetClosestPointRot(pointID, pc.rings[ringStart + 1], error);
	return error;
}