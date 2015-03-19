//
//  MSCKF.cpp
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#include "MSCKF.h"

#define SLIDING_WINDOW_SIZE 10
#define BODY_POSE_STATE_SIZE 10

MSCKF::MSCKF()
{
    slidingWindow = new VectorXf[SLIDING_WINDOW_SIZE];
    slidingWindow[0] = VectorXf(BODY_POSE_STATE_SIZE);
}

MSCKF::~MSCKF()
{
    delete[] slidingWindow;
}
