//
//  math_tool.cpp
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#include "math_tool.h"

/* my quaternion convention
    w = q(0);
    x = q(1);
    y = q(2);
    z = q(3);
 */


Matrix3f quaternion_to_R(const Vector4f& q)
{
    float n = q.norm();
    Vector4f nq = q / n;
    
    float w = nq(0);
    float x = nq(1);
    float y = nq(2);
    float z = nq(3);
    float w2 = w*w;
    float x2 = x*x;
    float y2 = y*y;
    float z2 = z*z;
    float xy = x*y;
    float xz = x*z;
    float yz = y*z;
    float wx = w*x;
    float wy = w*y;
    float wz = w*z;
    
    Matrix3f R(3,3);
    R(0,0) = w2+x2-y2-z2;
    R(1,0) = 2*(wz + xy);
    R(2,0) = 2*(xz - wy);
    R(0,1) = 2*(xy - wz);
    R(1,1) = w2-x2+y2-z2;
    R(2,1) = 2*(wx + yz);
    R(0,2) = 2*(wy + xz);
    R(1,2) = 2*(yz - wx);
    R(2,2) = w2-x2-y2+z2;
    return R;
}

Vector4f R_to_quaternion(const Matrix3f& R)
{
    Vector4f q(4);
    float S;
    float  tr = R(0,0) + R(1,1) + R(2,2);
    if (tr > 0)
    {
        S = sqrtf(tr + 1.0) * 2;
        q(0) = 0.25 * S;
        q(1) = (R(2,1) - R(1,2)) / S;
        q(2) = (R(0,2) - R(2,0)) / S;
        q(3) = (R(1,0) - R(0,1)) / S;
    }
    else if (R(0,0) > R(1,1) && R(0,0) > R(2,2))
    {
        S = sqrtf(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2;
        q(0) = (R(2,1) - R(1,2)) / S;
        q(1) = 0.25 * S;
        q(2) = (R(0,1) + R(1,0)) / S;
        q(3) = (R(0,2) + R(2,0)) / S;
    }
    else if (R(1,1) > R(2,2))
    {
        S = sqrtf(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2;
        q(0) = (R(0,2) - R(2,0)) / S;
        q(1) = (R(0,1) + R(1,0)) / S;
        q(2) = 0.25 * S;
        q(3) = (R(1,2) + R(2,1)) / S;
    }
    else
    {
        S = sqrtf(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2;
        q(0) = (R(1,0) - R(0,1)) / S;
        q(1) = (R(0,2) + R(2,0)) / S;
        q(2) = (R(1,2) + R(2,1)) / S;
        q(3) = 0.25 * S;
    }
    return q;
}


Matrix3f skew_mtx(const Vector3f& w)
{
    Matrix3f W;
    W <<     0, -w(2),  w(1),
    w(2),     0, -w(0),
    -w(1),  w(0),     0;
    return W;
}

Matrix4f omega_mtx(const Vector3f& w)
{
    Matrix4f omega;
    
    omega.block<3,3>(0,0) = - skew_mtx(w);
    omega.block<3,1>(0,3) = w;
    omega.block<1,3>(3,0) = - w.transpose();
    omega(3,3) = 0.0f;
    return omega;
}

// shelley thesis
// calculate small rotation using the fourth order Runge-Kutta method
Vector4f delta_quaternion(const Vector3f& w_prev, const Vector3f& w_curr, const float dt)
{
    Vector4f q, q0, k1, k2, k3, k4;
    q0 << 1.0f, 0.0f, 0.0f, 0.0f;
    
    k1 = 0.5f * omega_mtx(w_prev) * q0;
    k2 = 0.5f * omega_mtx(0.5*(w_prev+w_curr)) * (q0 + 0.5*dt*k1);
    k3 = 0.5f * omega_mtx(0.5*(w_prev+w_curr)) * (q0 + 0.5*dt*k2);
    k4 = 0.5f * omega_mtx(w_curr) * (q0 + dt*k3);
    
    q = q0 + (dt*(k1+2*k2+2*k3+k4))/6.0f;
    
    q = q / q.norm();
    
    return q;
}
