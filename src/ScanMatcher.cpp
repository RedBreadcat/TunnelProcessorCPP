#include "ScanMatcher.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include <random>
#include <algorithm>

using namespace std;

ScanMatcher::ScanMatcher()
{
}

//https://en.wikipedia.org/wiki/Random_sample_consensus
void ScanMatcher::DoRANSAC(PointCloud& pc)
{
    random_device rd;
    mt19937 rng(rd());
    uniform_int_distribution<int> uniformIntegers(0, 1079);

    //For each ring
    for (int i = 0; i < pc.rings.size() - 1; i++)
    {
        cout << "RING: " << i << endl;
        int it = 0;
        double bestError = HUGE_VALF;
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

            Eigen::Vector2d adjustment = DoICP(pc, i, possibleInliersForTesting);
            pc.rings[i + 1].moveToBeAligned = adjustment;
            //cout << "Adjustment: " << adjustment(0) << ", " << adjustment(1) << endl;

            //For every point which isn't in possibleInliersForTesting, see if when transformed they have a small error. If error is small, add to the vector
            int pointCount = pc.rings[i + 1].GetPointCount();
            for (int j = 0; j < pointCount; j++)
            {
                if (pc.rings[i].PointValid(j) && pc.rings[i + 1].PointValid(j))
                {
                    bool doesNotContainPoint = find(possibleInliersForTesting.begin(), possibleInliersForTesting.end(), j) == possibleInliersForTesting.end();  //Slow. Can build up a vector of points in above loop
                    if (doesNotContainPoint)
                    {
                        Eigen::Vector2d alignedPoint = pc.rings[i + 1].GetPointAligned(j);
                        Eigen::Vector2d refPoint = pc.rings[i].GetPointAligned(j);
                        double error = sqrt((refPoint[0]-alignedPoint[0])*(refPoint[0]-alignedPoint[0]) + (refPoint[1]-alignedPoint[1])*(refPoint[1]-alignedPoint[1]));

                        if (error < 20)   //If error is small
                        {
                            possibleInliersForTesting.push_back(j); //Rather than making an "alsoinliers" vector too, add directly to the possibleInliersForTesting given they'll be combined later anyway
                        }
                    }
                }
            }

            if (possibleInliersForTesting.size() > 400 + numTestPoints) //400: the minimum number of points required for this model to be considered good
            {
                adjustment = DoICP(pc, i, possibleInliersForTesting);    //Make a model using the new set of points with low error
                pc.rings[i + 1].moveToBeAligned = adjustment;

                //Calculate error of the model against all its points
                double accumulatedError = 0;
                for (int j = 0; j < possibleInliersForTesting.size(); j++)
                {
                    Eigen::Vector2d alignedPoint = pc.rings[i + 1].GetPointAligned(j);
                    Eigen::Vector2d refPoint = pc.rings[i].GetPointAligned(j);

                    accumulatedError += sqrt((refPoint[0]-alignedPoint[0])*(refPoint[0]-alignedPoint[0]) + (refPoint[1]-alignedPoint[1])*(refPoint[1]-alignedPoint[1]));
                }

                double averageError = accumulatedError / possibleInliersForTesting.size();
                if (averageError < bestError)
                {
                    bestAdjustment = adjustment;
                    bestError = averageError;
                }
            }
        }

        pc.rings[i + 1].moveToBeAligned = bestAdjustment;
        //cout << "Best adjustment: " << bestAdjustment(0) << ", " << bestAdjustment(1) << endl;
        //cout << "Best error: " << bestError << endl;
    }
}

//https://github.com/ethz-asl/libpointmatcher/blob/master/doc/icpWithoutYaml.md
Eigen::Vector2d ScanMatcher::DoICP(PointCloud& pc, int ringStart, vector<int> pointIDs)  //Returns the amount that each ring+1 point needs to move to be aligned
{
    typedef PointMatcher<double> PM;
    typedef PM::DataPoints DP;

	auto featuresReference = PM::Matrix(4, pointIDs.size());	//3 dimensions and padding. This huge matrix can be treated like an array.
    auto featuresNext = PM::Matrix(4, pointIDs.size());

	for (int i = 0; i < pointIDs.size(); i++)
	{
        int pointID = pointIDs[i];

        Eigen::Vector2d refPt = pc.rings[ringStart].GetPointAligned(pointID);
        Eigen::Vector2d nextPt = pc.rings[ringStart + 1].GetPointAligned(pointID);  //Note: at this stage, adjustment should be 0,0.

        featuresReference(0, i) = refPt[0]; //x
		featuresReference(1, i) = refPt[1]; //y
		featuresReference(2, i) = 0;        //z (todo)
        featuresReference(3, i) = 1;        //padding in order to make a standard 4x4 transformation matrix
        featuresNext(0, i) = nextPt[0];
		featuresNext(1, i) = nextPt[1];
		featuresNext(2, i) = 0;
		featuresNext(3, i) = 1;
	}


	PointMatcherIO<double>::LabelGenerator labelGenerator;
	labelGenerator.add("x");
	labelGenerator.add("y");
	labelGenerator.add("z");
    labelGenerator.add("pad");
	DP dpRef(featuresReference, labelGenerator.getLabels());
    DP dpNext(featuresNext, labelGenerator.getLabels());

    PM::ICP icp;
    PointMatcherSupport::Parametrizable::Parameters params;

    params["minDist"] = "1.0";
    PM::DataPointsFilter* minDist_read = PM::get().DataPointsFilterRegistrar.create("MinDistDataPointsFilter", params);
    params.clear();

    /*name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.05";
    PM::DataPointsFilter* rand_read = PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();*/

    params["minDist"] = "1.0";
    PM::DataPointsFilter* minDist_ref = PM::get().DataPointsFilterRegistrar.create("MinDistDataPointsFilter", params);
    params.clear();

    /*name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.05";
    PM::DataPointsFilter* rand_ref = PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();*/

    // Prepare matching function
    params["knn"] = "1";
    params["epsilon"] = "3.16";
    PM::Matcher* kdtree = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);
    params.clear();

    // Prepare outlier filters
    /*name = "TrimmedDistOutlierFilter";
    params["ratio"] = "0.75";
    PM::OutlierFilter* trim = PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();*/

    // Prepare error minimization
    PM::ErrorMinimizer* pointToPoint = PM::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer");

    // Prepare outlier filters
    params["maxIterationCount"] = "150";
    PM::TransformationChecker* maxIter = PM::get().TransformationCheckerRegistrar.create("CounterTransformationChecker", params);
    params.clear();

    params["minDiffRotErr"] = "0.001";
    params["minDiffTransErr"] = "0.01";
    params["smoothLength"] = "4";
    PM::TransformationChecker* diff = PM::get().TransformationCheckerRegistrar.create("DifferentialTransformationChecker", params);
    params.clear();

    // Prepare inspector
    PM::Inspector* nullInspect = PM::get().InspectorRegistrar.create("NullInspector");

    // Prepare transformation
    PM::Transformation* rigidTrans = PM::get().TransformationRegistrar.create("RigidTransformation");

    // Build ICP solution
    //icp.readingDataPointsFilters.push_back(minDist_read);
    //icp.readingDataPointsFilters.push_back(rand_read);

    //icp.referenceDataPointsFilters.push_back(minDist_ref);
    //icp.referenceDataPointsFilters.push_back(rand_ref);

    icp.matcher.reset(kdtree);
        
    //icp.outlierFilters.push_back(trim);
        
    icp.errorMinimizer.reset(pointToPoint);

    icp.transformationCheckers.push_back(maxIter);
    icp.transformationCheckers.push_back(diff);
        
    icp.inspector.reset(nullInspect);

    icp.transformations.push_back(rigidTrans);

    // Compute the transformation to express data in ref
    PM::TransformationParameters transformationParameters = icp(dpNext, dpRef);

    return Eigen::Vector2d(transformationParameters(0, 3), transformationParameters(1, 3));


    //cout << "RING: " << ringStart << ", POINTS: " << pointIDs.size() << endl << transformationParameters << endl << endl;
    //Transform data to express it in terms of ref
    //DP data_out(dpNext);
    //icp.transformations.apply(data_out, transformationParameters);
}