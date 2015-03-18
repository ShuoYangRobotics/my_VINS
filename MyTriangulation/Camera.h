//
//  Camera.h
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#ifndef MyTriangulation_Camera_h
#define MyTriangulation_Camera_h
#include <Eigen/Dense>

class DistortCamera {
    int width;
    int height;
    int nRows;
    int nCols;
    
    float fx;
    float fy;
    
    float ox;
    float oy;
    
    float focus_length;
    
    // distortion parameters, opencv k1, k2, p1, p2, k3
    float k1;
    float k2;
    float p1;
    float p2;
    float k3;
    
    Eigen::Matrix3f K;
    Eigen::Vector2f optical;
    Eigen::Matrix2f focusMtx;
    
public:
    DistortCamera();
    void setImageSize(float _height, float _width);
    void setIntrinsicMtx(float _fx, float _fy, float _ox, float _oy);
    void setDistortionParam(float _k1, float _k2, float _p1, float _p2, float _k3);
    
    Eigen::Vector2f h(Eigen::Vector3f ptr);
    
    Eigen::MatrixXf Jh(Eigen::Vector3f ptr);
};

#endif
