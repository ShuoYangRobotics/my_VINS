//
//  FeatureManager.h
//  MyTriangulation
//
//  Created by Yang Shuo on 26/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#ifndef __MyTriangulation__FeatureManager__
#define __MyTriangulation__FeatureManager__

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

struct FeatureInformation
{
    Vector3f point;
    FeatureInformation(Vector3f _feature_point);
};
struct SlideState
{
    Vector4f q;
    Vector3f p;
    Vector3f v;
};

class FeatureRecord
{
    public:
        const int feature_id;
        vector<FeatureInformation> feature_points;
    
        bool is_used;
        bool is_lost;
        bool is_outlier;
        int start_frame;
    
        FeatureRecord(int _feature_id, const Vector3f &_feature_point):
        feature_id(_feature_id), feature_points {FeatureInformation(_feature_point)}
        {
            is_used = false;
            is_lost = false;
            is_outlier = false;
        }
};

class FeatureManager
{
    public:
        FeatureManager();
    
    private:
        list<SlideState> slidingWindow;  // body pose sliding window, each with dimension 10
        list<FeatureRecord> feature_list;
        int current_frame;
    
    
};

#endif /* defined(__MyTriangulation__FeatureManager__) */
