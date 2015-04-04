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
    //nominalState.segment(0, 4) = Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
    errorState = VectorXf::Zero(ERROR_STATE_SIZE);
    extrinsicP = VectorXf::Zero(3);
    
    fullNominalState = VectorXf::Zero(NOMINAL_STATE_SIZE+3);
    fullErrorState = VectorXf::Zero(ERROR_STATE_SIZE+3);
    
    phi = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    errorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    fullErrorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE+3, ERROR_STATE_SIZE+3);
    fullErrorCovariance.block<3,3>(ERROR_STATE_SIZE,ERROR_STATE_SIZE) = Matrix3f::Identity();
    
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
    int errorStateLength = (int)fullErrorState.size();
    
    errorState = VectorXf::Zero(ERROR_STATE_SIZE);
    errorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    
    // CAUTION: their sizes are not fixed
    fullErrorState = VectorXf::Zero(errorStateLength);
    fullErrorCovariance = MatrixXf::Identity(errorStateLength, errorStateLength);
    phi = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
}

void MSCKF::setNoiseMatrix(float dgc, float dac, float dwgc, float dwac)
{
    Nc.block<3,3>(0, 0) = Matrix3f::Identity()*dgc;
    Nc.block<3,3>(6, 6) = Matrix3f::Identity()*dac;
    Nc.block<3,3>(9, 9) = Matrix3f::Identity()*dwgc;
    Nc.block<3,3>(12, 12) = Matrix3f::Identity()*dwac;
}

void MSCKF::setNominalState(Vector4f q, Vector3f p, Vector3f v, Vector3f bg, Vector3f ba, Vector3f pbc)
{
    fullNominalState.segment(0, 4) = q;
    fullNominalState.segment(4, 3) = p;
    fullNominalState.segment(7, 3) = v;
    fullNominalState.segment(10, 3) = bg;
    fullNominalState.segment(13, 3) = ba;
    fullNominalState.segment(16, 3) = pbc;
    
    nominalState = fullNominalState.head(NOMINAL_STATE_SIZE);
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
    std::cout << spatial_rotation << std::endl;
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
    
    std::cout << "phi is" << std::endl;
    std::cout << phi << std::endl;
    
    errorCovariance = phi*(errorCovariance+0.5*dt*Nc)*phi.transpose() + Nc;
    
    fullErrorCovariance.block<ERROR_STATE_SIZE, ERROR_STATE_SIZE>(0, 0) = errorCovariance;
    
    return;
}

void MSCKF::processImage(const vector<pair<int, Vector3d>> &image, vector<pair<Vector3d, Vector3d>> &corres)
{

    //bool ok = f_manager.addFeatureCheckParallax(frame_count, image, frame_count < WINDOW_SIZE);

}

void MSCKF::addSlideState()
{
    SlideState newState;
    int nominalStateLength = (int)fullNominalState.size();
    int errorStateLength = (int)fullErrorState.size();
    
    VectorXf tmpNominal = VectorXf::Zero(nominalStateLength+NOMINAL_POSE_STATE_SIZE);
    VectorXf tmpError   = VectorXf::Zero(errorStateLength+ERROR_POSE_STATE_SIZE);
    MatrixXf tmpCovariance = MatrixXf::Zero(errorStateLength+ERROR_POSE_STATE_SIZE,errorStateLength+ERROR_POSE_STATE_SIZE);
    MatrixXf Jpi = MatrixXf::Zero(9, errorStateLength);
    
    tmpNominal.head(nominalStateLength) = fullNominalState;
    tmpNominal.segment(nominalStateLength,NOMINAL_POSE_STATE_SIZE) = fullNominalState.head(NOMINAL_POSE_STATE_SIZE);
    fullNominalState = tmpNominal;
    
    tmpError.head(errorStateLength) = fullErrorState;
    tmpError.segment(errorStateLength,ERROR_POSE_STATE_SIZE) = fullErrorState.head(ERROR_POSE_STATE_SIZE);
    fullErrorState = tmpError;
    
    newState.q = fullNominalState.head(4);
    newState.p = fullNominalState.segment(4,3);
    newState.v = fullNominalState.segment(7,3);
    
    slidingWindow.push_back(newState);
    
    Jpi.block<3,3>(0,0) = Matrix3f::Identity(3,3);
    Jpi.block<3,3>(3,3) = Matrix3f::Identity(3,3);
    Jpi.block<3,3>(6,6) = Matrix3f::Identity(3,3);
    tmpCovariance.block(0,0, errorStateLength, errorStateLength) = fullErrorCovariance;
    tmpCovariance.block(errorStateLength,0,9,errorStateLength) = Jpi*fullErrorCovariance;
    tmpCovariance.block(0,errorStateLength,errorStateLength,9) = fullErrorCovariance*Jpi.transpose();
    tmpCovariance.block<ERROR_POSE_STATE_SIZE,ERROR_POSE_STATE_SIZE>(errorStateLength,errorStateLength) =
        Jpi*fullErrorCovariance*Jpi.transpose();
    
    fullErrorCovariance = tmpCovariance;
    
    
}

// CAUTION: make sure this function is not called at wrong time, and make sure that the index is valid
//          this function does not check index validity yet
void MSCKF::removeSlideState(int index, int total)
{
    //TODO: check total with (nominalStateLength-NOMINAL_POSE_STATE_SIZE-3)/NOMINAL_POSE_STATE_SIZE
    
    int nominalStateLength = (int)fullNominalState.size();
    int errorStateLength = (int)fullErrorState.size();
    
    VectorXf tmpNominal = VectorXf::Zero(nominalStateLength-NOMINAL_POSE_STATE_SIZE);
    VectorXf tmpError   = VectorXf::Zero(errorStateLength-ERROR_POSE_STATE_SIZE);
    MatrixXf tmpCovariance = MatrixXf::Zero(errorStateLength-ERROR_POSE_STATE_SIZE,errorStateLength-ERROR_POSE_STATE_SIZE);
    
    tmpNominal.head(NOMINAL_STATE_SIZE+3) = fullNominalState.head(NOMINAL_STATE_SIZE+3);
    tmpError.head(ERROR_STATE_SIZE+3) = fullErrorState.head(ERROR_STATE_SIZE+3);
    
    for (int i=0; i<total;i++)
    {
        if (i == index)
        {
            continue;
        }
        else if (i < index)
        {
            tmpNominal.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE) =
                fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE);
            tmpError.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE) =
                fullErrorState.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE);
        }
        else
        {
            tmpNominal.segment(NOMINAL_STATE_SIZE+3+(i-1)*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE) =
                fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE);
            tmpError.segment(ERROR_STATE_SIZE+3+(i-1)*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE) =
                fullErrorState.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE);
        }
    }
    
    fullNominalState = tmpNominal;
    fullErrorState = tmpError;
    
    int position_index; // the position of state with index in the matrix
    position_index = ERROR_STATE_SIZE + 3 + index * ERROR_POSE_STATE_SIZE;
    
    tmpCovariance.block(0, 0, position_index, position_index) =
        fullErrorCovariance.block(0, 0, position_index, position_index);
    
    
    tmpCovariance.block(0, position_index,
                        position_index,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE) =
        fullErrorCovariance.block(0, position_index+ERROR_POSE_STATE_SIZE,
                                  position_index,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE);
    
    tmpCovariance.block(position_index,0,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                        position_index) =
        fullErrorCovariance.block(position_index+ERROR_POSE_STATE_SIZE,0,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                                  position_index);
    
    tmpCovariance.block(position_index,position_index,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE) =
        fullErrorCovariance.block(position_index+ERROR_POSE_STATE_SIZE,position_index+ERROR_POSE_STATE_SIZE,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE);
    fullErrorCovariance = tmpCovariance;
    
    std::list<SlideState>::iterator itr;
    itr = slidingWindow.begin();
    for (int idx = 0; idx<index;idx++)
    {
        itr++;
    }
    slidingWindow.erase(itr);
}


void MSCKF::printNominalState(bool is_full)
{
    int nominalStateLength = (int)fullNominalState.size();
    
    if (is_full == true)
    {
        std::cout<<"full nominal state is"<<std::endl;
        for (int i = 0; i < nominalStateLength; i++)
        {
            std::cout<< fullNominalState(i);
            if (i!=NOMINAL_STATE_SIZE-1)
                std::cout<<" ";
            else
                std::cout<<"|";
        }
        std::cout<<std::endl;
    }
    else
    {
        std::cout<<"nominal state is"<<std::endl;
        for (int i = 0; i < NOMINAL_STATE_SIZE; i++)
        {
            std::cout<< fullNominalState(i) << " ";
        }
        std::cout<<std::endl;
    }

}

void MSCKF::printErrorState(bool is_full)
{
    int errorStateLength = (int)fullErrorState.size();
    
    if (is_full == true)
    {
        std::cout<<"full error state is"<<std::endl;
        for (int i = 0; i < errorStateLength; i++)
        {
            std::cout<< fullErrorState(i);
            if (i!=ERROR_STATE_SIZE-1)
                std::cout<<" ";
            else
                std::cout<<"|";
        }
        std::cout<<std::endl;
    }
    else
    {
        std::cout<<"error state is"<<std::endl;
        for (int i = 0; i < ERROR_STATE_SIZE; i++)
        {
            std::cout<< fullErrorState(i) << " ";
        }
        std::cout<<std::endl;
    }
    
}

void MSCKF::printSlidingWindow()
{
    std::list<SlideState>::iterator itr;
    std::cout<<"sliding state is"<<std::endl;
    
    for(itr=slidingWindow.begin();itr!=slidingWindow.end();itr++)
    {
        std::cout<< itr->q(0) << " ";
        std::cout<< itr->q(1) << " ";
        std::cout<< itr->q(2) << " ";
        std::cout<< itr->q(3) << " ";
        std::cout<< itr->p(0) << " ";
        std::cout<< itr->p(1) << " ";
        std::cout<< itr->p(2) << " ";
        std::cout<< itr->v(0) << " ";
        std::cout<< itr->v(1) << " ";
        std::cout<< itr->v(2) << "|";
    }
    std::cout<<std::endl;
}

void MSCKF::printErrorCovariance(bool is_full)
{
    if (is_full)
    {
        std::cout<<"full error covariance is ";
        std::cout<<"(R: " << fullErrorCovariance.rows() <<", C: " << fullErrorCovariance.cols()<<")"<<std::endl;
        std::cout<<fullErrorCovariance<<std::endl;
    }
    else
    {
        std::cout<<"error covariance is "<<std::endl;
        std::cout<<errorCovariance<<std::endl;
    }
}
    