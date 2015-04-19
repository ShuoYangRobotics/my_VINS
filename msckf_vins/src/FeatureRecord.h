//
//  FeatureManager.h
//  MyTriangulation
//
//  Created by Yang Shuo on 26/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#ifndef __MyTriangulation__FeatureRecord__
#define __MyTriangulation__FeatureRecord__

#include <list>
#include <algorithm>
#include <vector>
#include <map>
#include <numeric>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

class FeatureInformation
{
    public:
        Vector3d point;
        FeatureInformation(const Vector3d &_point): point(_point)
        {}
};

class FeatureRecord
{
    public:
        vector<FeatureInformation> feature_points;
    
        bool is_used;
        bool is_lost;
        bool is_outlier;
        int start_frame;
    
        FeatureRecord()
        {
            start_frame = -1;
            is_used = false;
            is_lost = false;
            is_outlier = false;
        }
    
    
        FeatureRecord(int _start_frame, const Vector3d &_feature_point):
        feature_points {FeatureInformation(_feature_point)}
        {
            start_frame = _start_frame;
            is_used = false;
            is_lost = false;
            is_outlier = false;
        }
};


#endif /* defined(__MyTriangulation__FeatureRecord__) */
