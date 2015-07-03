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
    int nRows;
    int nCols;
    
    double fx;
    double fy;
    
    double ox;
    double oy;
        
    // distortion parameters, opencv k1, k2, p1, p2, k3
    double k1;
    double k2;
    double p1;
    double p2;
    double k3;
    
    Eigen::Vector2d optical;
    Eigen::Matrix2d focusMtx;
    
public:
    int width;
    int height;

    DistortCamera();
    void setImageSize(double _height, double _width);
    void setIntrinsicMtx(double _fx, double _fy, double _ox, double _oy);
    void setDistortionParam(double _k1, double _k2, double _p1, double _p2, double _k3);
    
    Eigen::Vector2d h(Eigen::Vector3d ptr);
    
    Eigen::MatrixXd Jh(Eigen::Vector3d ptr);
    
    Eigen::Vector3d triangulate(Eigen::MatrixXd measure, Eigen::MatrixXd pose);
};

#endif
