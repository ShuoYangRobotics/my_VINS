#ifndef _CALIB_H_
#define _CALIB_H_

#include <iostream>
#include <Eigen/Dense>
#include "camera.h"
using namespace Eigen;
using namespace std;

class Calib
{
public:
	/* Camera */
    Camera camera;                // Camera

    /* Camera Position */
    Quaterniond CI_q;            // Rotation from camera to IMU coordinates. [unit quaternion]
    Vector3d C_p_I;              // Position of inertial frame in camera coordinates [m,m,m]
    //
    // Physical properties
    //
    double g;           // Gravitational acceleration [m/s^2]
    double delta_t;     // Time between IMU data [s]
    double image_imu_offset_t; // Time delay for images [s]
    //
    // Noise levels
    //
    /* IMU TODO: add correct units */
    double sigma_gc, sigma_ac,    // Standard deviation of IMU noise [rad/s, m/s^2]
           sigma_wgc, sigma_wac; // Standard deviation of IMU random walk noise [rad/s, m/s^2]
    /* Camera */
    double sigma_Im; // Image noise
    
    //
    // Options
    //
    unsigned int maxFrame; // Maximum frames in FIFO
    unsigned int minFrame; // Minimun frames in FIFO (is smaller before the first minFrames). Is used if features are only added when there is a match

    Calib();

    friend ostream& operator<<(ostream& out, const Calib& calib);
};

#endif //_CALIB_H_

