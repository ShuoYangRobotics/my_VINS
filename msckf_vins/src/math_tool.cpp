//
//  math_tool.cpp
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#include "math_tool.h"

/* my quaternion convention
    double w = nq(0);
    double x = nq(1);
    double y = nq(2);
    double z = nq(3);
 */


Matrix3d quaternion_to_R(const Vector4d& q)
{
    double n = q.norm();
    Vector4d nq = q / n;
    
    double w = nq(0);
    double x = nq(1);
    double y = nq(2);
    double z = nq(3);
    double w2 = w*w;
    double x2 = x*x;
    double y2 = y*y;
    double z2 = z*z;
    double xy = x*y;
    double xz = x*z;
    double yz = y*z;
    double wx = w*x;
    double wy = w*y;
    double wz = w*z;
    
    Matrix3d R(3,3);
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

Vector4d R_to_quaternion(const Matrix3d& R)
{
    Vector4d q(4);
    double S;
    double  tr = R(0,0) + R(1,1) + R(2,2);
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


Matrix3d skew_mtx(const Vector3d& w)
{
    Matrix3d W;
    W <<     0, -w(2),  w(1),
    w(2),     0, -w(0),
    -w(1),  w(0),     0;
    return W;
}

Matrix4d omega_mtx(const Vector3d& w)
{
    Matrix4d omega;
    
    omega.block<3,3>(0,0) = - skew_mtx(w);
    omega.block<3,1>(0,3) = w;
    omega.block<1,3>(3,0) = - w.transpose();
    omega(3,3) = 0.0f;
    return omega;
}

// shelley thesis
// calculate small rotation using the fourth order Runge-Kutta method
Vector4d delta_quaternion(const Vector3d& w_prev, const Vector3d& w_curr, const double dt)
{
    Vector4d q, q0, k1, k2, k3, k4;
    q0 << 1.0f, 0.0f, 0.0f, 0.0f;
    
    k1 = 0.5f * omega_mtx(w_prev) * q0;
    k2 = 0.5f * omega_mtx(0.5*(w_prev+w_curr)) * (q0 + 0.5*dt*k1);
    k3 = 0.5f * omega_mtx(0.5*(w_prev+w_curr)) * (q0 + 0.5*dt*k2);
    k4 = 0.5f * omega_mtx(w_curr) * (q0 + dt*k3);
    
    q = q0 + (dt*(k1+2*k2+2*k3+k4))/6.0f;
    
    q = q / q.norm();
    
    return q;
}

Vector4d quaternion_correct(Vector4d q, Vector3d d_theta)
{
    Vector4d corrected_q;
    Quaterniond qf(
                   q(0),
                   q(1),
                   q(2),
                   q(3)
                   );
    Quaterniond dq(
                   1,
                   0.5*d_theta(0),
                   0.5*d_theta(1),
                   0.5*d_theta(2)
                   );
    dq.w() = 1 - dq.vec().transpose() * dq.vec();
    
    qf = (qf * dq).normalized();
    corrected_q <<
    qf.x(),qf.y(),qf.z(),qf.w();
    
    return corrected_q;
}
