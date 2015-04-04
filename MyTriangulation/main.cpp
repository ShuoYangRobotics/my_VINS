//
//  main.cpp
//  MyTriangulation
//
//  Created by Yang Shuo on 18/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#include <iostream>
#include <map>
#include <Eigen/Dense>

#include "Camera.h"
#include "math_tool.h"
#include "MSCKF.h"
#include "FeatureManager.h"
using namespace Eigen;

// global variable to test
FeatureManager myManager;
MSCKF my_kf;
void featureManagerTest();
void KFtest();

int main(int argc, const char * argv[]) {
    DistortCamera cam;
    cam.setImageSize(480, 752);
    cam.setIntrinsicMtx(365.07984, 365.12127, 381.0196, 254.4431);
    cam.setDistortionParam(-2.842958e-1,
                            8.7155025e-2,
                           -1.4602925e-4,
                           -6.149638e-4,
                           -1.218237e-2);
    
    Vector3f ptr;
    ptr << 3.5f, 3.35f, 6.0f;
    Vector2f z = cam.h(ptr);
    MatrixXf Jacob_h = cam.Jh(ptr);
    
    std::cout << "projected point is " << z << std::endl;
    std::cout << "Jacobian matrix is " << std::endl << Jacob_h << std::endl;
    
    std::cout << "Jacobian matrix has cols " << Jacob_h.cols() << std::endl;
    
    Vector4f q;
    Matrix3f R;
    q << 1, 2, 3, 4;
    q = q/q.norm();
    
    R = quaternion_to_R(q);
    
    q = Vector4f(1,0,0,0);
    
    std::cout << "q is " << q << std::endl;
    std::cout << "R is " << R << std::endl;
    
    Matrix4f omega;
    omega = omega_mtx(ptr);
    
    std::cout << "omega is " << omega << std::endl;
    
    
    MatrixXf measure(2,5);
    MatrixXf pose(7,5);
    measure <<
    497.6229,  493.1491,  489.0450,  478.8504,  466.8726,
    371.0596,  366.5853,  362.4808,  352.2850,  340.3059
    ;
    pose<<
    0.777986724316206,   0.838539246498940,   0.775194258937213,   0.831502026804405,   0.770011014887736,
    0.625646233952092,   0.489266453739099,   0.435311151332881,   0.225961749519687,   0.059184118324008,
    -0.057475618563915,  -0.239729575410618,  -0.457796966390153,  -0.507489573463504,  -0.635279684146888,
    0,                   0,                   0,                   0,                   0,
    1.098427340189829,   5.357568053111257,   9.092272676622571,  11.936962300957290,  13.613178885567471,
    13.956842672263791,  12.934313455158014,  10.645683518400434,   7.314979906023285,   3.268235093982677,
    1.414213562373095,  -1.414213562373095,   1.414213562373095,  -1.414213562373094,   1.414213562373095
    ;
    
    
    std::cout << "measure is " << std::endl << measure << std::endl;
    std::cout << "pose is " << std::endl<< pose << std::endl;
    
    Vector3f ptr_pose = cam.triangulate(measure, pose);
    std::cout << "ptr_pose is " << std::endl<< ptr_pose << std::endl;
    
    
    Quaternionf q2(1.0, 0.0, 0.0, 0.0);
    Quaternionf q3(1.0, 0.0, 0.0, 0.0);
    Quaternionf dq(1,
                   0.7,
                   0.7,
                   0.7);
    dq.w() = 1 - dq.vec().transpose()*dq.vec();
    q2 = (q2*dq).normalized();
    std::cout << "q2 is " << std::endl<< q2.w() << q2.x() << q2.y() << q2.z() << std::endl;
    
    Vector3f w_prev(1.4,1.4,1.4);
    Vector3f w_curr(1.4,1.4,1.4);
    Vector4f dq2 = delta_quaternion(w_prev, w_curr, 1.0f);
    dq.w() = dq2(3);
    dq.x() = -dq2(0);
    dq.y() = -dq2(1);
    dq.z() = -dq2(2);
    q3 = (q3 * dq).normalized();
    std::cout << "q3 is " << std::endl<< q3.w() << q3.x() << q3.y() << q3.z() << std::endl;
    
    //Matrix3f ff = q3.vec();
    std::map<int,std::string> mymap;
    mymap[100]="an element";
    mymap[200]="another element";
    mymap[300]=mymap[200];
    
    std::cout << "mymap[100] is " << mymap[100] << '\n';
    std::cout << "mymap[200] is " << mymap[200] << '\n';
    std::cout << "mymap[300] is " << mymap[300] << '\n';
    std::cout << "mymap[301] is " << mymap[301] << '\n';
    
    std::cout << "find 301"  << mymap.find(301)->second << '\n';
    std::cout << "find 302"  << mymap.find(302)->second << '\n';
    
    std::cout << "mymap now contains " << mymap.size() << " elements.\n";

    //featureManagerTest();
    KFtest();
    return 0;
}

void KFtest()
{
    Vector4f q(1.0f, 0.0f, 0.0f, 0.0f);
    Vector3f p(3.0f, 3.0f, 4.5f);
    Vector3f v(1.3f, 1.4f, 1.5f);
    Vector3f bg(0.0f ,0.0f, 0.0f);
    Vector3f ba(0.0f ,0.0f, 0.0f);
    Vector3f pbc(10.0f ,0.0f, 0.0f);
    my_kf.setNominalState(q, p, v, bg, ba, pbc);
    my_kf.printNominalState(true);
    my_kf.printErrorCovariance(true);
    my_kf.addSlideState();
    my_kf.printNominalState(true);
    my_kf.printErrorCovariance(true);
    my_kf.printSlidingWindow();
    
    v = Vector3f(1.9f, 2.0f, 2.1f);
    my_kf.setNominalState(q, p, v, bg, ba, pbc);
    
    my_kf.printNominalState(true);
    my_kf.printErrorCovariance(true);
    my_kf.addSlideState();
    my_kf.printSlidingWindow();
    
    my_kf.printNominalState(true);
    my_kf.printErrorCovariance(true);
    my_kf.removeSlideState(0, 2);
    
    my_kf.printNominalState(true);
    my_kf.printErrorCovariance(true);
    my_kf.printSlidingWindow();
    my_kf.removeSlideState(0, 1);
    
    my_kf.printErrorCovariance(true);
    my_kf.processIMU(1.0f, Vector3f(0.9f, 0.9f, 0.9f), Vector3f(0.5f, 0.0f, 0.0f));
    my_kf.processIMU(1.4f, Vector3f(0.9f, 0.9f, 0.9f), Vector3f(0.5f, 0.0f, 0.0f));
    my_kf.printErrorCovariance(true);
}

void foo()
{
    SlideState state;
    Vector4f q(1.0f, 0.5f, 0.5f, 0.5f);
    Vector3f p(3.0f, 3.0f, 4.5f);
    Vector3f v(1.3f, 1.4f, 1.5f);
    state.p = p;
    state.q = q;
    state.v = v;
    
    myManager.addSlideState(state);
}
void featureManagerTest()
{
    
    foo();
    myManager.debugOut();
}
