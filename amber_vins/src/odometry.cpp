#include "odometry.h"
#include <Eigen/Eigenvalues>

bool isPositiveDefinite(MatrixXd m, bool to_print = false)
{
    assert ((m - m.transpose()).norm() < 0.1);

    EigenSolver<MatrixXd> es(m, false);
    VectorXcd eivals = es.eigenvalues();
    
    bool valid = true;
    for (int i = 0; i < eivals.rows(); i++) 
        if (eivals(i).real() < -1e-1) 
            valid = false;

    if (!valid || to_print)
        cout << eivals.transpose() << endl;
    return valid;
}

void CameraMeasurements::addToFeature(list<CameraMeas_t>::iterator& feature, const Vector2d& p)
{
    // Add feature 
    Matrix2Xd& z = feature->z;
    z.conservativeResize(NoChange, z.cols() + 1);
    z.rightCols<1>() = p;
    feature->isLost = false;
}

list<CameraMeas_t>::iterator CameraMeasurements::addNewFeature(int id)
{
    // Add a new entry
    CameraMeas_t z;
    list<CameraMeas_t>::iterator newFeature = meas.insert(meas.begin(), z);
    newFeature->z = Matrix2Xd(2, 0);
    newFeature->id = id;
    // lost (well empty)
    newFeature->isLost = true;

    return newFeature;
}

list<CameraMeas_t>::iterator CameraMeasurements::removeFeature(list<CameraMeas_t>::iterator& feature) 
{
    link.erase(feature->id);
    return meas.erase(feature);
}

void CameraMeasurements::addFeatures(const vector<pair<int, Vector2d>>& features)
{
    // Go through all measurements and mark all of them as lost
    for (auto& meas_it: meas) {
        meas_it.isLost = true;
    }
    for (auto& it: features)
    {
        if (link.find(it.first) == link.end())
            link[it.first] = addNewFeature(it.first);

        addToFeature(link[it.first], it.second);
    }
}

MSCKF::MSCKF(Calib* cal): Odometry(cal) 
{
    // init all states to 0;
    x = VectorXd::Zero(16);
    // init quaternion
    x.segment<4>(0) = Quaterniond(1, 0, 0, 0).coeffs();
    // init state to known
    sigma = MatrixXd::Identity(15, 15);

    // init delayed measurements
    I_a_prev = Vector3d(0, 0, 0);
    I_g_prev = Vector3d(0, 0, 0);

    initialized = false;
}

Quaterniond MSCKF::getQuaternion()
{
    return Quaterniond(x.segment<4>(0));
}

Vector3d MSCKF::getPosition()
{
    return x.segment<3>(0 + 4);
}

Vector3d MSCKF::getVelocity()
{
    return x.segment<3>(0 + 4 + 3);
}

// propagate
void MSCKF::propagate(const Vector3d& I_a_m, const Vector3d& I_g_m, bool propagateError)
{
    /*
    ** Reusable constants:
    */
    Vector3d G_g(0, 0, -calib->g);

    /*
    ** unpack state:
    */
    
    // Notation: [from][to]_[q/R]

    // Rotation from inertial to global coordinates
    Quaterniond IG_q(x.segment<4>(0));
    // Position (of inertial frame) in global coordinates
    Vector3d G_p(x.segment<3>(0 + 4));
    // Velocity (of inertial frame) in global coordinates
    Vector3d G_v(x.segment<3>(0 + 4 + 3));
    // Estimated gyro bias
    Vector3d b_g(x.segment<3>(0 + 4 + 3 + 3));
    // Esitmated accelerometer bias
    Vector3d b_a(x.segment<3>(0 + 4 + 3 + 3 + 3));

    /*
    ** Calibrate
    */
    // Accelerometer
    Vector3d I_a = I_a_m - b_a;
    // Gyro
    Vector3d I_g = I_g_m - b_g;
    
    if (!initialized)
    {
        I_a_prev = I_a;
        I_g_prev = I_g;
        initialized = true;
    }
    
    /*
    ** Propagate IMU
    */
    // Rotation
    Vector4d q0 = Vector4d(0, 0, 0, 1);
    Vector4d k1 = Omega(I_g_prev) * q0 / 2.0;
    Vector4d k2 = Omega((I_g_prev + I_g) / 2.0) * (q0 + calib->delta_t / 2.0 * k1) / 2.0;
    Vector4d k3 = Omega((I_g_prev + I_g) / 2.0) * (q0 + calib->delta_t / 2.0 * k2) / 2.0;
    Vector4d k4 = Omega(I_g) * (q0 + calib->delta_t * k3) / 2.0;

    Quaterniond I1I_q(q0 + calib->delta_t / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4));

    Quaterniond I1G_q = Quaterniond((Matrix4d::Identity() + Omega(I_g * calib->delta_t) / 2.0) * IG_q.coeffs());
    I1G_q.normalize();
    I1I_q = IG_q.inverse() * I1G_q;
 
//    I1I_q.normalize();
//    Quaterniond I1G_q = IG_q * I1I_q;

    // Translation
    Vector3d G_a = I1G_q * I_a + G_g;

    Vector3d s = calib->delta_t / 2.0 * (I1I_q * I_a + I_a_prev);

    Vector3d y = calib->delta_t / 2.0 * s;

    Vector3d G_v1 = G_v + IG_q * s + G_g * calib->delta_t;

    Vector3d G_p1 = G_p + G_v * calib->delta_t + IG_q * y + G_g * calib->delta_t * calib->delta_t / 2.0;

    /*
    ** Repack state
    */
    // Rotation from inertial to global coordinates
    x.segment<4>(0) = I1G_q.coeffs();
    // Position (of inertial frame) in global coordinates
    x.segment<3>(0 + 4) = G_p1;
    // Velocity (of inertial frame) in global coordinates
    x.segment<3>(0 + 4 + 3) = G_v1;
    // No change to biases

    /*
    ** Propagate error
    */

    // aliases
    Matrix3d R = IG_q.toRotationMatrix();
    Matrix3d R1 = I1G_q.toRotationMatrix();
    Matrix3d R_ = R.transpose();
    Matrix3d R1_ = R1.transpose();

    Matrix3d Phi_qq = Matrix3d::Identity();
    Matrix3d Phi_pq = -crossMat(R_ * y);
    Matrix3d Phi_vq = -crossMat(R_ * s);

    Matrix3d Phi_qbg = -calib->delta_t / 2.0 * (R1_ + R_);
    Matrix3d Phi_vbg = calib->delta_t * calib->delta_t / 4.0 * (crossMat(G_a - G_g) * (R1_ + R_));
    Matrix3d Phi_pbg = calib->delta_t / 2.0 * Phi_vbg;

    Matrix3d Phi_qba = Matrix3d::Zero(); // assuming no gravitational effect on gyro
    Matrix3d Phi_vba = -calib->delta_t / 2.0 * (R1_ + R_);
    Matrix3d Phi_pba = calib->delta_t / 2.0 * Phi_vba;

    Matrix<double, 15, 15> Phi_I;
    Phi_I <<
                  Phi_qq,     Matrix3d::Zero(),                      Matrix3d::Zero(),              Phi_qbg,              Phi_qba,
                  Phi_pq, Matrix3d::Identity(), calib->delta_t * Matrix3d::Identity(),              Phi_pbg,              Phi_pba,
                  Phi_vq,     Matrix3d::Zero(),                  Matrix3d::Identity(),              Phi_vbg,              Phi_vba,
        Matrix3d::Zero(),     Matrix3d::Zero(),                      Matrix3d::Zero(), Matrix3d::Identity(),     Matrix3d::Zero(),
        Matrix3d::Zero(),     Matrix3d::Zero(),                      Matrix3d::Zero(),     Matrix3d::Zero(), Matrix3d::Identity();

    Matrix<double, 15, 15> N_c;
    Matrix<double, 15, 15> Q_d;
    Matrix<double, 15, 1> N_c_diag;
    N_c_diag <<
           calib->sigma_gc * calib->sigma_gc * Vector3d(1, 1, 1),
           Vector3d(0, 0, 0),
           calib->sigma_ac * calib->sigma_ac * Vector3d(1, 1, 1),
           calib->sigma_wgc * calib->sigma_wgc * Vector3d(1, 1, 1),
           calib->sigma_wac * calib->sigma_wac * Vector3d(1, 1, 1);
    N_c = N_c_diag.asDiagonal();
    Q_d = calib->delta_t / 2.0 * Phi_I * N_c * Phi_I.transpose() + N_c;

    // cout << Phi_I << endl;
    // cout << sigma << endl;
    // cout << "Phi_I * sigma * Phi_I^T" << endl << Phi_I * sigma.block<15, 15>(0, 0) * Phi_I.transpose() << endl;

    // do the update
    sigma.block<15, 15>(0, 0) = Phi_I * sigma.block<15, 15>(0, 0) * Phi_I.transpose() + Q_d;
    sigma.block(0, 15, 15, sigma.cols() - 15) = Phi_I * sigma.block(0, 15, 15, sigma.cols() - 15);
    sigma.block(15, 0, sigma.rows() - 15, 15) = sigma.block(0, 15, 15, sigma.cols() - 15).transpose();

    if (!isPositiveDefinite(sigma, true))
    {
        cout << "not positve definite" << endl;
        abort();
    }
//    static int cnt = 0;
//    if (cnt++ > 200)
        //abort();
    /*
    ** update delayed variables
    */
    I_a_prev = I_a;
    I_g_prev = I_g;
}

// TODO: -Propagate state <- used for pure prediction
// TODO: -Propagate sigma <- no use for this

// augment state
void MSCKF::augmentState(void)
{
    /*
    ** Resize state and sigma, so new data can fit
    */

    // State
    x.conservativeResize(x.rows() + ODO_STATE_FRAME_SIZE, NoChange);
    x.block<ODO_STATE_FRAME_SIZE, 1>(x.rows() - ODO_STATE_FRAME_SIZE, 0) =
        x.block<ODO_STATE_FRAME_SIZE, 1>(0, 0);

    // Covariance
    sigma.conservativeResize(sigma.rows() + ODO_SIGMA_FRAME_SIZE,
                             sigma.cols() + ODO_SIGMA_FRAME_SIZE
                            );
    /*
    ** Fast way of doing:
    ** J = [ I_9, 0_(9,6), 0_(9,n*9) ]
    ** where the frame size in sigma is 9 and n is the previous number of frames
    ** Then do:
    ** Sigma = [ Sigma, Sigma*J', J*Sigma, J*Sigma*J']
    ** Since J is the identitymatrix and a lot of zeros, this is straight-up copying:
    */
    sigma.block<ODO_SIGMA_FRAME_SIZE, ODO_SIGMA_FRAME_SIZE>(sigma.rows() - ODO_SIGMA_FRAME_SIZE, sigma.cols() - ODO_SIGMA_FRAME_SIZE) =
        sigma.block<ODO_SIGMA_FRAME_SIZE, ODO_SIGMA_FRAME_SIZE>(0, 0);
    sigma.block(0, sigma.cols() - ODO_SIGMA_FRAME_SIZE, sigma.rows() - ODO_SIGMA_FRAME_SIZE, ODO_SIGMA_FRAME_SIZE) =
        sigma.block(0, 0, sigma.rows() - ODO_SIGMA_FRAME_SIZE, ODO_SIGMA_FRAME_SIZE);
    sigma.block(sigma.rows() - ODO_SIGMA_FRAME_SIZE, 0, ODO_SIGMA_FRAME_SIZE, sigma.cols() - ODO_SIGMA_FRAME_SIZE) =
        sigma.block(0, 0, ODO_SIGMA_FRAME_SIZE, sigma.cols() - ODO_SIGMA_FRAME_SIZE);        
}

// remove n old states
void MSCKF::removeOldStates(int n)
{
    //
    // Skip if n < 1
    //
    if (n < 1)
        return;
    //
    // Remove at most the number of frames we have
    //
    if (n > (x.rows() - ODO_STATE_SIZE) / ODO_STATE_FRAME_SIZE)
        n = (x.rows() - ODO_STATE_SIZE) / ODO_STATE_FRAME_SIZE;

    /*
    ** Remove the n oldest frames from the state and covariance
    */
    x.segment(ODO_STATE_SIZE, x.rows() - ODO_STATE_SIZE - n * ODO_STATE_FRAME_SIZE) =
        x.segment(ODO_STATE_SIZE + n * ODO_STATE_FRAME_SIZE, x.rows() - ODO_STATE_SIZE - n * ODO_STATE_FRAME_SIZE);
    x.conservativeResize(x.rows() - n * ODO_STATE_FRAME_SIZE, NoChange);

    sigma.block(ODO_SIGMA_SIZE, 0, sigma.rows() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE, sigma.cols()) =
        sigma.block(ODO_SIGMA_SIZE + n * ODO_SIGMA_FRAME_SIZE, 0, sigma.rows() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE, sigma.cols());

    sigma.block(0, ODO_SIGMA_SIZE, sigma.rows(), sigma.cols() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE) =
        sigma.block(0, ODO_SIGMA_SIZE + n * ODO_SIGMA_FRAME_SIZE, sigma.rows(), sigma.cols() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE);

    sigma.conservativeResize(sigma.rows() - n * ODO_SIGMA_FRAME_SIZE, sigma.cols() - n * ODO_SIGMA_FRAME_SIZE);
}

// performUpdate
void MSCKF::performUpdate(const VectorXd& delta_x)
{
    //
    // apply feedback
    //

    // To inertial state
    // IG_q
    Quaterniond delta_IG_q(1, delta_x(0) / 2.0, delta_x(1) / 2.0, delta_x(2) / 2.0);
    Quaterniond IG_q = delta_IG_q * Quaterniond(x.block<4, 1>(0, 0));
    IG_q.normalize();
    x.block<4, 1>(0, 0) = IG_q.coeffs();
    // G_p
    x.block<3, 1>(0 + 4, 0) += delta_x.block<3, 1>(0 + 3, 0);
    // G_v
    x.block<3, 1>(0 + 4 + 3, 0) += delta_x.block<3, 1>(0 + 3 + 3, 0);
    // b_g
    x.block<3, 1>(0 + 4 + 3 + 3, 0) += delta_x.block<3, 1>(0 + 3 + 3 + 3, 0);
    // b_a
    x.block<3, 1>(0 + 4 + 3 + 3 + 3, 0) += delta_x.block<3, 1>(0 + 3 + 3 + 3 + 3, 0);

    // to all the frames
    for (int i = 0; i < (x.rows() - ODO_STATE_SIZE) / ODO_STATE_FRAME_SIZE; i++) 
    {
        unsigned int frameStart = ODO_STATE_SIZE + i * ODO_STATE_FRAME_SIZE;
        unsigned int delta_frameStart = ODO_SIGMA_SIZE + i * ODO_SIGMA_FRAME_SIZE;
        Quaterniond delta_IiG_q(1,
                                delta_x(delta_frameStart + 0) / 2.0,
                                delta_x(delta_frameStart + 1) / 2.0,
                                delta_x(delta_frameStart + 2) / 2.0
                               );
        Quaterniond IiG_q = delta_IiG_q * Quaterniond(x.block<4, 1>(frameStart + 0, 0));
        IiG_q.normalize();
        x.block<4, 1>(frameStart + 0, 0) = IiG_q.coeffs();
        // G_p_i
        x.block<3, 1>(frameStart + 0 + 4, 0) += delta_x.block<3, 1>(delta_frameStart + 0 + 3, 0);
        // G_v_i
        x.block<3, 1>(frameStart + 0 + 4 + 3, 0) += delta_x.block<3, 1>(delta_frameStart + 0 + 3 + 3, 0);
    }
}

// updateCamera
void MSCKF::updateCamera(CameraMeasurements& cameraMeasurements)
{
    cout << *this << endl;

    //
    // Append current state to frame FIFO
    //
    this->augmentState();
    cout << "after augment state" << endl;
    cout << *this << endl;

    // initialize H0 and r0
    VectorXd r0(0, 0);
    MatrixXd H0(0, sigma.cols());
    //
    // Get max length of "living" feature r0 and H0
    //
    unsigned int longestLiving = 0;
    for (list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end();) {

        // Enforce maximum age of features. TODO: figure out if this is really the best way
        if (!meas_j->isLost && (meas_j->z.cols() >= calib->maxFrame)) 
        {
            meas_j->isLost = true;
            meas_j->z.conservativeResize(NoChange, meas_j->z.cols() - 1);
        }

        unsigned int n = meas_j->z.cols();

        if (meas_j->isLost) {
            // If more that, or 3 points, use for update
            if (n >= 3) {
                Matrix4Xd CG_q(4, n);
                Matrix3Xd G_p_C(3, n);
                
                for (int i = 0; i < n; i++) 
                {
                    unsigned int frameStart = x.rows() - ODO_STATE_FRAME_SIZE * (n - i + 1);
                    // Get inertial frame state at that time:
                    Quaterniond IiG_q(x.block<4, 1>(frameStart + 0, 0));
                    Quaterniond CiG_q = IiG_q * calib->CI_q;
                    // Calculate camera state
                    Vector3d G_p_Ii = x.block<3, 1>(frameStart + 4, 0);
                    Vector3d G_p_Ci = G_p_Ii - CiG_q * calib->C_p_I;

                    CG_q.col(i) = CiG_q.coeffs();
                    G_p_C.col(i) = G_p_Ci;
                }

                double r_norm;
                Vector3d G_p_f = calib->camera.triangulate(CG_q, G_p_C, meas_j->z, &r_norm);

                if (r_norm < 100)
                {
                    // cout << "r_norm: " << r_norm << endl;
                    // cout << "feature id: " << meas_j->id << endl;
                    // cout << "G_z_f:" << endl << meas_j->z << endl;
                    // cout << "G_p_f:" << G_p_f.transpose() << endl;

                    // If not a clear outlier:
                    if (isfinite(G_p_f(0)) && isfinite(G_p_f(1)) && isfinite(G_p_f(2))) {

//                        cout << sigma << endl;
                        // Marignalize:
                        VectorXd r0j = VectorXd(n * 2 - 3);
                        MatrixXd H0j = MatrixXd(n * 2 - 3, sigma.cols());
                        this->marginalize(meas_j->z, G_p_f, r0j, H0j);
                        

                        // TODO: Check if inlier
                        // cout << "marginalize" << endl;

                        if (isInlinerCamera(r0j, H0j)) {
                            //cout << r0j << endl;
                            //cout << H0j << endl;

                            // Add to huge H0 and r0 matrix
                            H0.conservativeResize(H0.rows() + H0j.rows(), NoChange);
                            r0.conservativeResize(r0.rows() + r0j.rows(), NoChange);
                            H0.bottomRows(H0j.rows()) = H0j;
                            r0.bottomRows(r0j.rows()) = r0j;
                        }
                    }
                }
            }
            // in any case, remove it and advance
            meas_j = cameraMeasurements.removeFeature(meas_j);
        } else {
            // Set longest living
            longestLiving = (n > longestLiving) ? n : longestLiving;
            // Skip and advance
            ++meas_j;
        }
    }
    //
    // TODO: QR decomposition (skipped for now)
    //

    //
    // Calculate kalmangain and apply update
    //
    // Only if we have measurements
    if (r0.rows() > 0) 
    {
//        cout << "r0.rows():" << r0.rows() <<  endl;
        // Image noise
        MatrixXd R_q = MatrixXd::Identity(r0.rows(), r0.rows()) * calib->sigma_Im * calib->sigma_Im;

        // Kalman gain
        MatrixXd K = sigma * H0.transpose() * (H0 * sigma * H0.transpose() + R_q).inverse();
        // cout << "r0: " << r0 << endl;

        // cout << "eigen of sigma " << endl;
        // isPositiveDefinite(sigma, true);

        // cout << "eigen of H0 * H0.transpose() " << endl;
        // isPositiveDefinite(H0 * H0.transpose(), true);

        // cout << "eigen of H0 * sigma * H0.transpose()" << endl;
        // isPositiveDefinite(H0 * sigma * H0.transpose(), true);

        // cout << "eigen of H0 * sigma * H0.transpose() inverse" << endl;
        // isPositiveDefinite((H0 * sigma * H0.transpose()).inverse(), true);


//        cout << "H0: " << H0 << endl;
//        cout << "K: " << K << endl;
        // Update to be appled to state
        VectorXd delta_x = K * r0;

  //      cout << "sigma" << endl << sigma << endl;

        // Update covariance
        MatrixXd A = MatrixXd::Identity(K.rows(), H0.cols()) - K * H0;
        sigma = A * sigma * A.transpose() + K * R_q * K.transpose();

        //
        // apply feedback
        //
        this->performUpdate(delta_x);

//        cout << "sigma" << endl << sigma << endl;
//        cout << "delta_x: " << endl << delta_x.transpose() << endl;

//        abort();

        // if (delta_x.norm() > 1)
        //     abort();
    }

    //
    // Make sure sigma is symetric
    //
    sigma = (sigma + sigma.transpose()) / 2;

    //
    // Remove all old and unused frames
    //

    //
    // Make sure that we let atleast "minFrame" frames live
    //
    if (longestLiving < calib->minFrame)
        longestLiving = calib->minFrame;
    this->removeOldStates((x.rows() - ODO_STATE_SIZE) / ODO_STATE_FRAME_SIZE - longestLiving);
}

// -marginalize
void MSCKF::marginalize(const Matrix2Xd& z, const Vector3d& G_p_f, Ref<VectorXd> r0, Ref<MatrixXd> H0)
{
    //
    // calculate residuals and
    //
    unsigned int n = z.cols();
    VectorXd r(n * 2);
    MatrixX3d H_f(n * 2, 3);
    MatrixXd H_x = MatrixXd::Zero(n * 2, sigma.cols());

    for (int i = 0; i < n; i ++) {
        //
        // calculate camera position:
        //
        // Get index of start of this frame in state
        unsigned int frameStart = x.rows() - ODO_STATE_FRAME_SIZE * (n - i + 1);
        // Get inertial frame state at that time:
        Quaterniond IiG_q(x.block<4, 1>(frameStart + 0, 0));
        Quaterniond CiG_q = IiG_q * calib->CI_q;
        // Calculate camera state
        Vector3d G_p_Ii = x.block<3, 1>(frameStart + 4, 0);
        Vector3d G_p_Ci = G_p_Ii - CiG_q * calib->C_p_I;

        // Calculate feature position in camera frame
        Vector3d C_p_f = CiG_q.inverse() * (G_p_f - G_p_Ci);

        r.block<2, 1>(i * 2, 0) = z.col(i) - calib->camera.cameraProject(C_p_f);
        H_f.block<2, 3>(i * 2, 0) = calib->camera.jacobianH(C_p_f) * CiG_q.toRotationMatrix().transpose();
        H_x.block<2, 9>(i * 2, H_x.cols() - ODO_SIGMA_FRAME_SIZE * (n - i + 1)) <<
                H_f.block<2, 3>(i * 2, 0) * crossMat(G_p_f - G_p_Ii), -H_f.block<2, 3>(i * 2, 0), Matrix<double, 2, 3>::Zero();
    }

    // Find left null-space
    JacobiSVD<MatrixXd> svd(H_f, Eigen::ComputeFullU);
    MatrixXd A = svd.matrixU().rightCols(n * 2 - 3).transpose();

    if (n * 2 - 3 != A.rows()) {
        cout << "A: " << A.rows() << "x" << A.cols() << endl;
        cout << "z: " << z.rows() << "x" << z.cols() << endl;
        cout << "r0: " << r0.rows() << "x" << r0.cols() << endl;
        cout << "H0: " << H0.rows() << "x" << H0.cols() << endl;
        cout << "H_f: " << H_f.rows() << "x" << H_f.cols() << endl;
        cout << "H_x: " << H_x.rows() << "x" << H_x.cols() << endl;
        cout << "H_f" << H_f << endl;
        cout << "H_x" << H_x << endl;
    }

    // Marginalize
    r0 = A * r;
    H0 = A * H_x;
}


// TODO: -isInlinerCamera
bool MSCKF::isInlinerCamera(const VectorXd& r0, const MatrixXd& H0)
{
    if (!isPositiveDefinite(sigma))
    {
        cout << "not positve definite" << endl;
        abort();
    }
    double gamma = r0.transpose() * (H0 * sigma * H0.transpose()).inverse() * r0;

    cout << "gamma: " << gamma << endl;

    if (gamma < 0)
        abort();

    if (sizeof(chi2Inv) / sizeof(*chi2Inv) > r0.rows()) 
        return gamma <= chi2Inv[ r0.rows() ] && gamma >= 0;
    else
        return gamma >= 0 && gamma <= 1074;
}

// print
ostream& operator<<(ostream& out, const MSCKF& msckf)
{
    out <<
           "IG_q: " << msckf.x.segment<4>(0).transpose() << "\n" <<
           "G_p: " << msckf.x.segment<3>(0 + 4).transpose() << "\n" <<
           "G_v: " << msckf.x.segment<3>(0 + 4 + 3).transpose() << "\n" <<
           "b_g: " << msckf.x.segment<3>(0 + 4 + 3 + 3).transpose() << "\n" <<
           "b_a: " << msckf.x.segment<3>(0 + 4 + 3 + 3 + 3).transpose() << "\n";

    for (int i = 0; i < (msckf.x.rows() - ODO_STATE_SIZE) / ODO_STATE_FRAME_SIZE; i++)
    {
        unsigned int frameStart = ODO_STATE_SIZE + i * ODO_STATE_FRAME_SIZE;
        out <<
            "I" << i << "G_q: " << msckf.x.segment<4>(frameStart + 0).transpose() << "\n" <<
            "G_p_" << i << ": " << msckf.x.segment<3>(frameStart + 0 + 4).transpose() << "\n" <<
            "G_v_" << i << ": " << msckf.x.segment<3>(frameStart + 0 + 4 + 3).transpose() << "\n";
    }

    return out;
}
