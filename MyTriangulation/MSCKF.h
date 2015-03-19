//
//  MSCKF.h
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#ifndef __MyTriangulation__MSCKF__
#define __MyTriangulation__MSCKF__

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class MSCKF {
    VectorXf nominalState;  // dimension 4 + 3 + 3 + 3 + 3 = 16
    VectorXf errorState;    // dimension 3 + 3 + 3 + 3 + 3 = 15
    VectorXf extrinsicP;    // p_bc, dimension 3
    VectorXf* slidingWindow;  // body pose sliding window, each with dimension 10
    
public:
    MSCKF();
    ~MSCKF();
};

#endif /* defined(__MyTriangulation__MSCKF__) */
