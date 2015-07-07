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
    return Quaterniond(q).matrix();
}

Vector4d R_to_quaternion(const Matrix3d& R)
{
    return Quaterniond(R).coeffs();
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
Quaterniond delta_quaternion(const Vector3d& w_prev, const Vector3d& w_curr, const double dt)
{
    Vector4d q, q0, k1, k2, k3, k4;
    q0 << 0.0f, 0.0f, 0.0f, 1.0f;
    
    k1 = 0.5f * omega_mtx(w_prev) * q0;
    k2 = 0.5f * omega_mtx(0.5*(w_prev+w_curr)) * (q0 + 0.5*dt*k1);
    k3 = 0.5f * omega_mtx(0.5*(w_prev+w_curr)) * (q0 + 0.5*dt*k2);
    k4 = 0.5f * omega_mtx(w_curr) * (q0 + dt*k3);
    
    q = q0 + (dt*(k1+2*k2+2*k3+k4))/6.0f;
    
    q = q / q.norm();

    return Quaterniond(q);
}

Vector4d quaternion_correct(Vector4d q, Vector3d d_theta)
{
    Quaterniond qf(q);
    Quaterniond dq(
                   1,
                   0.5*d_theta(0),
                   0.5*d_theta(1),
                   0.5*d_theta(2)
                   );
    dq.w() = 1 - dq.vec().transpose() * dq.vec();
    
    qf = (dq * qf).normalized();
    return qf.coeffs();
}
