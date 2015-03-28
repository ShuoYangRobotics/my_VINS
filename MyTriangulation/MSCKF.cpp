//
//  MSCKF.cpp
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#include "MSCKF.h"
#include "math_tool.h"
#include "g_param.h"

static Vector3f g(0.0f, 0.0f, -9.81f);

MSCKF::MSCKF()
{
    //slidingWindow = new VectorXf[SLIDING_WINDOW_SIZE];
    //slidingWindow[0] = VectorXf(BODY_POSE_STATE_SIZE);
    
    nominalState = VectorXf::Zero(NOMINAL_STATE_SIZE);
    errorState = VectorXf::Zero(ERROR_STATE_SIZE);
    extrinsicP = VectorXf::Zero(3);
    errorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    Nc = MatrixXf::Zero(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    setNoiseMatrix(0.1f, 0.1f, 0.1f, 0.1f);
    
    current_time = -1.0f;
}

MSCKF::~MSCKF()
{
    //delete[] slidingWindow;
}

void MSCKF::resetError()
{
    errorState = VectorXf::Zero(ERROR_STATE_SIZE);
    errorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    phi = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
}

void MSCKF::setNoiseMatrix(float dgc, float dac, float dwgc, float dwac)
{
    Nc.block<3,3>(0, 0) = Matrix3f::Identity()*dgc;
    Nc.block<3,3>(6, 6) = Matrix3f::Identity()*dac;
    Nc.block<3,3>(9, 9) = Matrix3f::Identity()*dwgc;
    Nc.block<3,3>(12, 12) = Matrix3f::Identity()*dwac;
}

void MSCKF::processIMU(float t, Vector3f linear_acceleration, Vector3f angular_velocity)
{
    Vector4f small_rotation;
    Matrix3f d_R, prev_R, average_R, phi_vbg;
    Vector3f s_hat, y_hat;
    Vector3f tmp_vel, tmp_pos;
    // read nomial state to get variables
    spatial_quaternion = nominalState.segment(0, 4);
    spatial_position = nominalState.segment(4, 3);
    spatial_velocity = nominalState.segment(7, 3);
    gyro_bias = nominalState.segment(10, 3);
    acce_bias = nominalState.segment(13, 3);
    
    spatial_rotation = quaternion_to_R(spatial_quaternion);
    
    if (current_time < 0.0f)
    {
        current_time = t;
        prev_w = angular_velocity - gyro_bias;
        prev_a = linear_acceleration - acce_bias;
        return;
    }
    
    float dt = t - current_time;
    
    current_time = t;
    curr_w = angular_velocity - gyro_bias;
    curr_a = linear_acceleration - acce_bias;
    
    //calculate q_B{l+1}B{l}
    small_rotation = delta_quaternion(prev_w, curr_w, dt);
    d_R = quaternion_to_R(small_rotation);
    
    // defined in paper P.49
    s_hat = 0.5f * dt * (d_R.transpose()*curr_a + prev_a);
    y_hat = 0.5f * dt * s_hat;
    
    /* update nominal state */
    tmp_vel = spatial_velocity + spatial_rotation.transpose()*s_hat + g*dt;
    tmp_pos = spatial_position + spatial_velocity*dt
                               + spatial_rotation.transpose()*y_hat + 0.5f*g*dt*dt;
    
    spatial_velocity = tmp_vel;
    spatial_position = tmp_pos;
    prev_R = spatial_rotation;
    spatial_rotation = d_R*spatial_rotation;
    
    /* propogate error covariance */
    average_R = prev_R.transpose()+spatial_rotation.transpose();
    //1. phi_pq
    phi.block<3,3>(3,0) = -skew_mtx(prev_R*y_hat);
    //2. phi_vq
    phi.block<3,3>(6,0) = -skew_mtx(prev_R*s_hat);
    //3. one bloack need to times dt;
    phi.block<3,3>(3,6) = Matrix3f::Identity()*dt;
    //4. phi_qbg
    phi.block<3,3>(0,9) = -0.5f*dt*average_R;
    //5. phi_vbg
    phi_vbg = 0.25f*dt*dt*(skew_mtx(spatial_rotation.transpose()*curr_a)*average_R);
    phi.block<3,3>(6,9) = phi_vbg;
    //6. phi_pbg
    phi.block<3,3>(3,9) = 0.5f*dt*phi_vbg;
    
    //7. phi_vba
    phi.block<3,3>(6,12) = -0.5f*dt*average_R;
    //8. phi_pba
    phi.block<3,3>(3,12) = -0.25f*dt*dt*average_R;
    
    errorCovariance = phi*(errorCovariance+0.5*dt*Nc)*phi.transpose() + Nc;
    
    return;
}

void MSCKF::processImage(const vector<pair<int, Vector3d>> &image, vector<pair<Vector3d, Vector3d>> &corres)
{

    //bool ok = f_manager.addFeatureCheckParallax(frame_count, image, frame_count < WINDOW_SIZE);

}
    