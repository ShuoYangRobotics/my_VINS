//
//  MSCKF.cpp
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#include "MSCKF.h"

#define SLIDING_WINDOW_SIZE 10

#define NOMINAL_STATE_SIZE   16
#define ERROR_STATE_SIZE     15
#define BODY_POSE_STATE_SIZE 10

MSCKF::MSCKF()
{
    slidingWindow = new VectorXf[SLIDING_WINDOW_SIZE];
    slidingWindow[0] = VectorXf(BODY_POSE_STATE_SIZE);
    
    current_time = -1.0f;
}

MSCKF::~MSCKF()
{
    delete[] slidingWindow;
}


void MSCKF::processIMU(float t, Vector3f linear_acceleration, Vector3f angular_velocity)
{
    if (current_time < 0.0f)
    {
        current_time = t;
        prev_w = angular_velocity;
        prev_a = linear_acceleration;
        return;
    }
    
    float dt = t - current_time;
    
    current_time = t;
    curr_w = angular_velocity;
    curr_a = linear_acceleration;
    
    return;
}