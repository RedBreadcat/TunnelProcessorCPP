#include "ScanMatcher.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include <random>
#include <algorithm>

using namespace std;

ScanMatcher::ScanMatcher()
{

}

void ScanMatcher::DoRANSAC(PointCloud& pc)
{
    random_device rd;
    mt19937 rng(rd());
    uniform_int_distribution<int> uniformIntegers(0, 1079);

    //For each ring
    for (int i = 0; i < pc.rings.size() - 1; i++)
    {
        //Select random points
        vector<int> possibleInliersForTesting;
        possibleInliersForTesting.reserve(numTestPoints);
        while (possibleInliersForTesting.size() < numTestPoints)
        {
            int pointID = uniformIntegers(rng);
            bool doesNotContainPoint = find(possibleInliersForTesting.begin(), possibleInliersForTesting.end(), pointID) == possibleInliersForTesting.end();
            if (pc.rings[i].points[pointID].valid && pc.rings[i + 1].points[pointID].valid && doesNotContainPoint) //If point in this and next ring are valid, and point hasn't been added before
            {
                possibleInliersForTesting.push_back(pointID);
            }
        }

        DoICP(pc, i, possibleInliersForTesting);   //todo: get model from this (the transformation matrix)
        //for every point which isn't in possibleInliersForTesting, see if when transformed they have a small error. If error is small, add to vector additionalInliers
        //if additionalInliers.size() > minimum_number_of_points_I_require_for_a_model_to_be_considered_good
        //{
            //DoICP with possibleInliersForTesting and additionalInliers
            //Calculate error of this new transformation matrix against possibleInliersForTesting and additionalInliers
            //if (error < bestError)
            //{
                //bestTransformMatrix = 
                //bestError = error;  
            //}
        //}
    }

}

//https://github.com/ethz-asl/libpointmatcher/blob/master/doc/icpWithoutYaml.md
void ScanMatcher::DoICP(PointCloud& pc, int ringStart, vector<int> pointIDs)
{
    typedef PointMatcher<double> PM;
    typedef PM::DataPoints DP;

	auto featuresReference = PM::Matrix(4, pointIDs.size());	//3 dimensions and padding. This huge matrix can be treated like an array.
    auto featuresNext = PM::Matrix(4, pointIDs.size());

	for (int i = 0; i < pointIDs.size(); i++)
	{
        int pointID = pointIDs[i];

        featuresReference(0, i) = pc.rings[ringStart].points[pointID].pos[0];   //x
		featuresReference(1, i) = pc.rings[ringStart].points[pointID].pos[1];   //y
		featuresReference(2, i) = 0;                                            //z (todo)
        featuresReference(3, i) = 1;                                            //padding in order to make a standard 4x4 transformation matrix
        featuresNext(0, i) = pc.rings[ringStart + 1].points[pointID].pos[0];    //x
		featuresNext(1, i) = pc.rings[ringStart + 1].points[pointID].pos[1];    //y
		featuresNext(2, i) = 0;                                                 //z (todo)
		featuresNext(3, i) = 1;                                                 //padding
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


    cout << "RING: " << ringStart << ", POINTS: " << pointIDs.size() << endl << transformationParameters << endl << endl;
    //Transform data to express it in terms of ref
    //DP data_out(dpNext);
    //icp.transformations.apply(data_out, transformationParameters);
}