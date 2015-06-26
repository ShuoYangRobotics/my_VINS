//
//  math_tool.h
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#ifndef __MyTriangulation__math_tool__
#define __MyTriangulation__math_tool__

#include <Eigen/Dense>
using namespace Eigen;
Matrix3d quaternion_to_R(const Vector4d& q);
Vector4d R_to_quaternion(const Matrix3d& R);
Matrix3d skew_mtx(const Vector3d& w);
Matrix4d omega_mtx(const Vector3d& w);
Quaterniond delta_quaternion(const Vector3d& w_prev, const Vector3d& w_curr, const double dt);
Vector4d quaternion_correct(Vector4d q, Vector3d d_theta);
#endif /* defined(__MyTriangulation__math_tool__) */
