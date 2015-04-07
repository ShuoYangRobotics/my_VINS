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

#include <ros/ros.h>
#include <ros/console.h>

using namespace Eigen;
using namespace std;

#include "FeatureRecord.h"
#include "Camera.h"

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
    float measure_noise;
    
    /* feature management */
    map<int, FeatureRecord> feature_record_dict;
    list<VectorXf>  residual_list;
    list<MatrixXf>  H_mtx_list;
    list<int>       H_mtx_block_size_list;
    
    
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
    
    /* camera */    
    DistortCamera cam;
    
    /* fixed rotation between camera and the body frame */
    Matrix3f R_cb;
    
public:
    MSCKF();
    ~MSCKF();
    
    void resetError();
    void setNominalState(Vector4f q, Vector3f p, Vector3f v, Vector3f bg, Vector3f ba);
    void setCalibParam(Vector3f p_cb, float fx, float fy, float ox, float oy, float k1, float k2, float p1, float p2, float k3);
    void setIMUCameraRotation(Matrix3f _R_cb);
    
    void setNoiseMatrix(float dgc, float dac, float dwgc, float dwac);
    void setMeasureNoise(float _noise);
    
    void correctNominalState(VectorXf delta);
    
    void processIMU(float t, Vector3f linear_acceleration, Vector3f angular_velocity);
    void processImage(const vector<pair<int, Vector3f>> &image);
    
    void addSlideState();
    void removeSlideState(int index, int total);
    void addFeatures(const vector<pair<int, Vector3f>> &image);
    void removeFrameFeatures(int index);
    void removeUsedFeatures();
    
    Vector2f projectPoint(Vector3f feature_pose, Matrix3f R_bg, Vector3f p_gb, Vector3f p_cb);
    void getResidualH(VectorXf& ri, MatrixXf& Hi, Vector3f feature_pose, MatrixXf measure, MatrixXf pose_mtx, int frame_offset);
    
    /* outputs */
    Vector4f getQuaternion();
    Matrix3f getRotation();
    Vector3f getPosition();
    Vector3f getVelocity();
    Vector3f getGyroBias();
    Vector3f getAcceBias();
    Vector3f getVIOffset();
    
    
    /* debug outputs */
    void printNominalState(bool is_full);
    void printErrorState(bool is_full);
    void printSlidingWindow();
    void printErrorCovariance(bool is_full);
};

#endif /* defined(__MyTriangulation__MSCKF__) */
