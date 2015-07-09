#ifndef _DATA_GENERATOR_H_
#define _DATA_GENERATOR_H_

#include <cstdlib>
#include <cmath>
#include <map>
#include <vector>
#include <random>
#include <iostream>
#include <Eigen/Dense>

#include "calib.h"
using namespace std;
using namespace Eigen;

class DataGenerator
{
public:
    static int const NUM_POINTS  = 200;
    static int const MAX_BOX     = 10;
    static int const RANGE1      = 15;
    static int const RANGE2      = 25;
    static int const MAX_HEIGHT  = 5;

    static int const FREQ        = 200;
    static int const IMU_PER_IMG = 50;

    static int const MAX_TIME    = 10;

    static int const FOV         = 90;
    static int const ROW         = 480;
    static int const COL         = 752;

public:  
    Calib* calib;
    
    DataGenerator(Calib* _calib);
    void update();
    double getTime();

    Vector3d getPoint(int i);
    vector<Vector3d> getCloud();
    Vector3d getPosition();          //G_p 
    Matrix3d getRotation();          //IG_R
    Vector3d getVelocity();          //G_v 

    Vector3d getIMUAngularVelocity();    //I_w body frame
    Vector3d getIMULinearAcceleration(); //I_a body frame

    vector<pair<int, Vector2d>> getImage();

private:
    Vector3d pts[NUM_POINTS];
    double t;

    map<int, int> before_feature_id;
    map<int, int> current_feature_id;
    int current_id;

    default_random_engine random_generator;
    normal_distribution<double> distribution;
};

#endif //_DATA_GENERATOR_H_