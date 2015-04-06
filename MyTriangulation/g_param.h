//
//  g_param.h
//  MyTriangulation
//
//  Created by Yang Shuo on 26/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#ifndef MyTriangulation_g_param_h
#define MyTriangulation_g_param_h


//#define DEBUG_FLAG

#ifndef DEBUG_FLAG

#define SLIDING_WINDOW_SIZE 10     // 4 + 3 + 3
#define NOMINAL_STATE_SIZE   16    // 4 + 3 + 3 + 3 + 3
#define ERROR_STATE_SIZE     15    // 3 + 3 + 3 + 3 + 3
#define NOMINAL_POSE_STATE_SIZE 10 // 4 + 3 + 3
#define ERROR_POSE_STATE_SIZE 9    // 3 + 3 + 3

#else

#define SLIDING_WINDOW_SIZE 2
#define NOMINAL_STATE_SIZE   5
#define ERROR_STATE_SIZE     4
#define NOMINAL_POSE_STATE_SIZE 3
#define ERROR_POSE_STATE_SIZE 2

#endif

#endif
