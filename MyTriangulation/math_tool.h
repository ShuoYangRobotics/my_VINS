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
Matrix3f quaternion_to_R(const Vector4f& q);
Vector4f R_to_quaternion(const Matrix3f& R);
Matrix3f skew_mtx(const Vector3f& w);
Matrix4f omega_mtx(const Vector3f& w);
Vector4f delta_quaternion(const Vector3f& w_prev, const Vector3f& w_curr, const float dt);
#endif /* defined(__MyTriangulation__math_tool__) */
