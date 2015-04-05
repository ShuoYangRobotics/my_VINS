//
//  MSCKF.h
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#ifndef __MyTriangulation__MSCKF__
#define __MyTriangulation__MSCKF__

#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

#include "FeatureManager.h"
class FeatureRecord;

struct SlideState
{
    Vector4f q;
    Vector3f p;
    Vector3f v;
};

class MSCKF
{
private:
    /* states */
    VectorXf nominalState;  // dimension 4 + 3 + 3 + 3 + 3 = 16
    VectorXf errorState;    // dimension 3 + 3 + 3 + 3 + 3 = 15
    VectorXf extrinsicP;    // p_bc, dimension 3
    
    VectorXf fullNominalState;
    VectorXf fullErrorState;
    
    list<SlideState> slidingWindow;

    /* covariance */
    MatrixXf errorCovariance;
    MatrixXf fullErrorCovariance;
    MatrixXf phi;
    
    /* noise matrix */
    MatrixXf Nc;
    
    /* feature management */
    map<int, FeatureRecord> feature_record_dict;
    list<pair<int, Vector3f>> triangulate_ptrs;
    
    
    float current_time;     // indicates the current time stamp
    int   current_frame;    // indicates the current frame in slidingWindow
    
    /* IMU measurements */
    Vector3f prev_w, curr_w;
    Vector3f prev_a, curr_a;
    
    /* nominal state variables used only for calculation */
    Vector4f spatial_quaternion; // q_BG
    Matrix3f spatial_rotation; // R_BG
    Vector3f spatial_position;
    Vector3f spatial_velocity;
    Vector3f gyro_bias;
    Vector3f acce_bias;
    
    
    
public:
    MSCKF();
    ~MSCKF();
    
    void resetError();
    void setNominalState(Vector4f q, Vector3f p, Vector3f v, Vector3f bg, Vector3f ba, Vector3f pbc);
    void setNoiseMatrix(float dgc, float dac, float dwgc, float dwac);
    void processIMU(float t, Vector3f linear_acceleration, Vector3f angular_velocity);
    void processImage(const vector<pair<int, Vector3d>> &image, vector<pair<Vector3d, Vector3d>> &corres);
    
    void addSlideState();
    void removeSlideState(int index, int total);
    void addFeatures(const vector<pair<int, Vector3d>> &image);
    void removeFrameFeatures(int index);
    void removeUsedFeatures();
    
    
    /* debug outputs */
    void printNominalState(bool is_full);
    void printErrorState(bool is_full);
    void printSlidingWindow();
    void printErrorCovariance(bool is_full);
};

#endif /* defined(__MyTriangulation__MSCKF__) */
