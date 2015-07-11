#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "calib.h"
#include "utils.h"
using namespace Eigen;
using namespace std;

#define ODO_MAX_FRAMES (5) // used for static allocation of state and sigma
#define ODO_STATE_SIZE (4 + 3 + 3 + 3 + 3)
#define ODO_STATE_FRAME_SIZE (4 + 3 + 3)
#define ODO_STATE_MAX_SIZE (ODO_STATE_SIZE + ODO_STATE_FRAME_SIZE * (ODO_MAX_FRAMES + 1))
#define ODO_SIGMA_SIZE (3 + 3 + 3 + 3 + 3)
#define ODO_SIGMA_FRAME_SIZE (3 + 3 + 3)
#define ODO_SIGMA_MAX_SIZE (ODO_SIGMA_SIZE + ODO_SIGMA_FRAME_SIZE * (ODO_MAX_FRAMES + 1))

typedef struct
{
    int id;
    Matrix2Xd z; // length is stored as number of elements in z
    bool isLost; // Set if it is no longer tracked in current frame
                 // TODO: add feature description here?
} CameraMeas_t;

class CameraMeasurements 
{
public:
    list<CameraMeas_t> meas;
    //
    // Variables to hold feature info
    //
    map<int, list<CameraMeas_t>::iterator> link;
    
    void addToFeature(list<CameraMeas_t>::iterator& feature, const Vector2d& p);
    list<CameraMeas_t>::iterator addNewFeature(int id);
    list<CameraMeas_t>::iterator removeFeature(list<CameraMeas_t>::iterator& feature);
    void addFeatures(const vector<pair<int, Vector2d>>& features);
};

/***
**
** General 6DOF odometry with gyro and accelerometer (+biases)
**
***/
class Odometry
{
public:
    //
    // Calibration object
    //
    Calib* calib;
    // init
    Odometry(Calib* cal): calib(cal) {}
};

class MSCKF: public Odometry
{
public:
    //
    // State
    //
    // Matrix<double,Dynamic,1,0,ODO_STATE_MAX_SIZE,1> x; // allocate statically
    VectorXd x;
    //
    // Covariance
    //
    // Matrix<double,Dynamic,Dynamic,0,ODO_SIGMA_MAX_SIZE,ODO_SIGMA_MAX_SIZE> sigma; // allocate statically
    MatrixXd sigma;
    //
    // Local variables for integration
    //
    Vector3d I_a_prev, I_g_prev;

    bool initialized;

public:
    MSCKF(Calib* cal);

    Quaterniond getQuaternion();
    Vector3d getPosition();
    Vector3d getVelocity();

    void propagate(const Vector3d& I_a_m, const Vector3d& I_g_m, bool propagateError = true);
    void updateCamera(CameraMeasurements& cameraMeasurements);
    void augmentState(void);
    void removeOldStates(int n);

    void marginalize(const Matrix2Xd& z, const Vector3d& G_p_f, Ref<VectorXd> r0, Ref<MatrixXd> H0);
    bool isInlinerCamera(const VectorXd& r0, const MatrixXd& H0);
    void performUpdate(const VectorXd& delta_x);

    // print
    friend ostream& operator<<(ostream& out, const MSCKF& msckf);
};

#endif //_ODOMETRY_H_
