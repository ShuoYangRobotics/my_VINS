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
#include <map>
#include <numeric>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

#include "MSCKF.h"

struct FeatureInformation
{
    Vector3f point;
    FeatureInformation(Vector3f _feature_point);
};

class FeatureRecord
{
    public:
        vector<FeatureInformation> feature_points;
    
        bool is_used;
        bool is_lost;
        bool is_outlier;
        int start_frame;
    
        FeatureRecord(const Vector3f &_feature_point):
        feature_points {FeatureInformation(_feature_point)}
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
        void addFeatures        (const vector<pair<int, Vector3d>> &image, SlideState _state);
    
        void addSlideState      (SlideState _state);
        void removeSlideState   (int index);
        void debugOut();
    
    private:
        list<SlideState> slidingWindow;  // body pose sliding window, each with dimension 10
        map<int, FeatureRecord> feature_record_dict;
        list<pair<int, Vector3f>> triangulate_ptrs;
        int current_frame;               // indicates the number of sliding state
    
    
};

#endif /* defined(__MyTriangulation__FeatureManager__) */
