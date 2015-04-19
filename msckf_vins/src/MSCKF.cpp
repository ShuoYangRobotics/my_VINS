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

static Vector3d g(0.0f, 0.0f, -9.8f);

MSCKF::MSCKF()
{
    fullNominalState = VectorXd::Zero(NOMINAL_STATE_SIZE + 3);
    
    phi = MatrixXd::Identity(ERROR_STATE_SIZE, ERROR_STATE_SIZE);
    errorCovariance = MatrixXd::Identity(ERROR_STATE_SIZE, ERROR_STATE_SIZE);
    fullErrorCovariance = MatrixXd::Identity(ERROR_STATE_SIZE + 3, ERROR_STATE_SIZE + 3);
    
    Nc = MatrixXd::Zero(ERROR_STATE_SIZE, ERROR_STATE_SIZE);
    setNoiseMatrix(0.1f, 0.1f, 0.1f, 0.1f);
    
    current_time = -1.0f;
    
    cam.setImageSize(480, 752);
    
    cam.setIntrinsicMtx(365.07984, 365.12127, 381.0196, 254.4431);
    cam.setDistortionParam(-2.842958e-1,
                           8.7155025e-2,
                           -1.4602925e-4,
                           -6.149638e-4,
                           -1.218237e-2);
    
    current_frame = -1;   // initially no frame
    
//    R_cb = Matrix3d::Identity();
    R_cb <<
        0, -1, 0,
        0,  0, 1,
       -1,  0, 0;
    
    fullNominalState.segment(16, 3) = Vector3d(-0.14, -0.02, 0.0);   //p_cb
}

MSCKF::~MSCKF()
{

}

void MSCKF::resetError()
{
}

void MSCKF::setNoiseMatrix(double dgc, double dac, double dwgc, double dwac)
{
    Nc.block<3,3>(0, 0)   = Matrix3d::Identity() * dgc;
    Nc.block<3,3>(6, 6)   = Matrix3d::Identity() * dac;
    Nc.block<3,3>(9, 9)   = Matrix3d::Identity() * dwgc;
    Nc.block<3,3>(12, 12) = Matrix3d::Identity() * dwac;
}

void MSCKF::setMeasureNoise(double _noise)
{
    measure_noise = _noise;
}

void MSCKF::setNominalState(Vector4d q, Vector3d p, Vector3d v, Vector3d bg, Vector3d ba)
{
    fullNominalState.segment(0, 4)  = q;
    fullNominalState.segment(4, 3)  = p;
    fullNominalState.segment(7, 3)  = v;
    fullNominalState.segment(10, 3) = bg;
    fullNominalState.segment(13, 3) = ba;
}

void MSCKF::setCalibParam(Vector3d p_cb, double fx, double fy, double ox, double oy, double k1, double k2, double p1, double p2, double k3)
{
    fullNominalState.segment(16, 3) = p_cb;
    cam.setIntrinsicMtx(fx, fy, ox, oy);
    cam.setDistortionParam(k1, k2, p1, p2, k3);
}

void MSCKF::setIMUCameraRotation(Matrix3d _R_cb)
{
    R_cb = _R_cb;
}

void MSCKF::correctNominalState(VectorXd delta)
{
    fullNominalState.segment(0, 4)  = quaternion_correct(fullNominalState.segment(0, 4), delta.segment(0, 3));
    fullNominalState.segment(4, 3)  = fullNominalState.segment(4, 3)  + delta.segment(3, 3);
    fullNominalState.segment(7, 3)  = fullNominalState.segment(7, 3)  + delta.segment(6, 3);
    fullNominalState.segment(10, 3) = fullNominalState.segment(10, 3) + delta.segment(9, 3);
    fullNominalState.segment(13, 3) = fullNominalState.segment(13, 3) + delta.segment(12, 3);
    fullNominalState.segment(16, 3) = fullNominalState.segment(16, 3) + delta.segment(15, 3);   //p_cb
//    cout << __FILE__ << ":" << __LINE__ <<endl;
//    printNominalState(true);
    // loop to correct sliding states
    std::list<SlideState>::iterator itr = slidingWindow.begin();
    for (int i=0; i<=current_frame; i++)
    {
        // TODO: check order of multiplication
        fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE, 4) =
            quaternion_correct(fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE, 4),
                               delta.segment(ERROR_STATE_SIZE+3 + i*ERROR_POSE_STATE_SIZE, 3));
        itr->q = fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE, 4);
        
        fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE+4, 3) =
            fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE+4, 3) +
            delta.segment(ERROR_STATE_SIZE+3 + i*ERROR_POSE_STATE_SIZE+3, 3);
        itr->p = fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE+4, 3);
        
        fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE+7, 3) =
            fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE+7, 3) +
            delta.segment(ERROR_STATE_SIZE+3 + i*ERROR_POSE_STATE_SIZE+6, 3);
        itr->v = fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE+7, 3);
        
        itr++;
    }
}

void MSCKF::processIMU(double t, Vector3d linear_acceleration, Vector3d angular_velocity)
{
    Vector4d small_rotation;
    Matrix3d d_R, prev_R, average_R, phi_vbg;
    Vector3d s_hat, y_hat;
    Vector3d tmp_vel, tmp_pos;
    // read nomial state to get variables
    spatial_quaternion = fullNominalState.segment(0, 4); //q_gb
    spatial_position = fullNominalState.segment(4, 3);
    spatial_velocity = fullNominalState.segment(7, 3);
    gyro_bias = fullNominalState.segment(10, 3);
    acce_bias = fullNominalState.segment(13, 3);
    
    Quaterniond spa(
      spatial_quaternion(0),
      spatial_quaternion(1),
      spatial_quaternion(2),
      spatial_quaternion(3)
    );
  
    spatial_rotation = spa;
    //spatial_rotation = quaternion_to_R(spatial_quaternion); //R_gb
    if (current_time < 0.0f)
    {
        current_time = t;
        prev_w = angular_velocity - gyro_bias;
        prev_a = linear_acceleration - acce_bias;
        return;
    }
    
    double dt = t - current_time;
    //cout << "dt: " << dt << endl;
    
    current_time = t;
    curr_w = angular_velocity - gyro_bias;
    curr_a = linear_acceleration - acce_bias;

    
    //calculate q_B{l+1}B{l}
    small_rotation = delta_quaternion(prev_w, curr_w, dt);
    d_R = quaternion_to_R(small_rotation);
    // defined in paper P.49
    //s_hat = 0.5f * dt * (d_R.transpose() * curr_a + prev_a);
    s_hat = dt *curr_a;
    y_hat = 0.5f * dt * s_hat;
    
    /* update nominal state */
    prev_R = spatial_rotation;

    Quaterniond dq(1,
                   curr_w(0) * dt / 2,
                   curr_w(1) * dt / 2,
                   curr_w(2) * dt / 2);
    dq.w() = 1 - dq.vec().transpose() * dq.vec();
    Quaterniond q(spatial_rotation);
    spatial_rotation = (q * dq).normalized();

    //spatial_position += spatial_velocity * dt + spatial_rotation * curr_a * dt * dt / 2;
    //spatial_velocity += spatial_rotation * curr_a * dt + g * dt;

    tmp_pos = spatial_position + spatial_velocity * dt
                               + spatial_rotation * y_hat + 0.5 * g * dt * dt;
    tmp_vel = spatial_velocity + spatial_rotation * s_hat + g * dt;
    spatial_velocity = tmp_vel;
    spatial_position = tmp_pos;

   // cout << "spatial rotaiton: "<< endl << spatial_rotation << endl;
   // cout << "curr_w" << endl << curr_w << endl;
   // cout << "curr_a" << endl << curr_a << endl;
   // cout << "spatal acc" << endl << spatial_rotation * s_hat / dt + g  << endl;
   // cout << "spatal vel" << endl << spatial_velocity  << endl;
   // cout << "dR"  << endl<< d_R << endl;
   // cout << "tmp_pos" << endl << tmp_pos << endl;
   // cout << "tmp_pos increament1" << endl << spatial_velocity * dt << endl;
   // cout << "tmp_pos increament2" << endl << y_hat << endl;
    
    //spatial_rotation = spatial_rotation*d_R.transpose(); //R_gb{l+1} = R_gb{l}*q_B{l}B{l+1}
    
    //cout << R_to_quaternion(spatial_rotation) << endl;
    spa = spatial_rotation;
    spatial_quaternion(0) = spa.w();
    spatial_quaternion(1) = spa.x();
    spatial_quaternion(2) = spa.y();
    spatial_quaternion(3) = spa.z();
    //fullNominalState.segment(0, 4) = R_to_quaternion(spatial_rotation); //q_gb
    fullNominalState.segment(0, 4) = spatial_quaternion; //q_gb
    fullNominalState.segment(4, 3) = spatial_position;
    fullNominalState.segment(7, 3) = spatial_velocity;

    // save prev
    prev_w = curr_w;
    prev_a = curr_a;
    
    /* propogate error covariance */
    average_R = prev_R+spatial_rotation;
    phi = MatrixXd::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    //1. phi_pq
    phi.block<3,3>(3,0) = -skew_mtx(prev_R * y_hat);
    //2. phi_vq
    phi.block<3,3>(6,0) = -skew_mtx(prev_R * s_hat);
    //3. one bloack need to times dt;
    phi.block<3,3>(3,6) = Matrix3d::Identity() * dt;
    //4. phi_qbg
    phi.block<3,3>(0,9) = -0.5f * dt * average_R;
    //5. phi_vbg
    phi_vbg = 0.25f * dt * dt * (skew_mtx(spatial_rotation * curr_a) * average_R);
    phi.block<3,3>(6,9) = phi_vbg;
    //6. phi_pbg
    phi.block<3,3>(3,9) = 0.5f * dt * phi_vbg;
    
    //7. phi_vba
    phi.block<3,3>(6,12) = -0.5f * dt * average_R;
    //8. phi_pba
    phi.block<3,3>(3,12) = -0.25f * dt * dt * average_R;
    
    //std::cout << "phi is" << std::endl;
    //std::cout << phi << std::endl;
    
    errorCovariance = phi * (errorCovariance + 0.5 * dt * Nc) * phi.transpose() + Nc;
    
    fullErrorCovariance.block<ERROR_STATE_SIZE, ERROR_STATE_SIZE>(0, 0) = errorCovariance;
    
    return;
}

void MSCKF::processImage(const vector<pair<int, Vector3d>> &image)
{
    printf("input feature: %lu\n", image.size());
    
    // init is_lost
    for (auto & item : feature_record_dict)
    {
        if (item.second.is_used == false)
        {
            item.second.is_lost = true;
        }
    }
    
    // add sliding state
    // removeSlideState if the window is full already
    if (current_frame == SLIDING_WINDOW_SIZE-1)
    {
        removeSlideState(1, SLIDING_WINDOW_SIZE);
        removeFrameFeatures(1);
        current_frame--;
    }
    
    addSlideState();
    ++current_frame;
    printf("current frame is %d\n", current_frame);
    addFeatures(image);     // is_lost modified here

    //check is_lost to get measurement
    MatrixXd measure_mtx;
    MatrixXd pose_mtx;
    Vector3d ptr_pose;
    
    // clear these two lists
    residual_list.clear();
    H_mtx_list.clear();
    H_mtx_block_size_list.clear();
    
    int num_measure = 0;
    int row_H = 0;
    for (auto & item : feature_record_dict)
    {
        if (item.second.is_lost == true)
        {
            if (current_frame - item.second.start_frame >= 3)
            {
                /* 1. prepare to do triangulation */
                std::vector<FeatureInformation>::iterator itr_f = item.second.feature_points.begin();
                std::list<SlideState>::iterator           itr_s = slidingWindow.begin();
                for (int i = 0; i < item.second.start_frame; i++)
                {
                    itr_s ++;
                }
                
                int num_frame = current_frame - item.second.start_frame;
                
                measure_mtx = MatrixXd::Zero(2, num_frame);
                pose_mtx = MatrixXd::Zero(7, num_frame);
                
                for (int i = item.second.start_frame; i < current_frame; i++)
                {
                    // construct measure
                    measure_mtx(0, i-item.second.start_frame) = itr_f->point.x();
                    measure_mtx(1, i-item.second.start_frame) = itr_f->point.y();
                
                    // construct pose
//                    Matrix3d R_gb, R_gc;
//                    Vector3d p_gb, p_gc;
//                    R_gb = quaternion_to_R(itr_s->q);
//                    p_gb = itr_s->p;
//                    
//                    R_gc = R_gb*R_cb.transpose();
//                    
//                    /* p_gc = p_gb + p_bc */
//                    /* p_bc = -R_cb^T * p_cb */
//                    p_gc = p_gb + -R_cb.transpose() * fullNominalState.segment(16, 3);
                    
//                    pose_mtx.block<4,1>(0, i-item.second.start_frame) = R_to_quaternion(R_gc);  // q_gc
//                    pose_mtx.block<3,1>(4, i-item.second.start_frame) = p_gc;                   // p_gc
                    pose_mtx.block<4,1>(0, i-item.second.start_frame) = itr_s->q;  // q_gb
                    pose_mtx.block<3,1>(4, i-item.second.start_frame) = itr_s->p;  // p_gb
                    
                    
                    itr_f++;
                    itr_s++;
                }
                
                ptr_pose = cam.triangulate(measure_mtx, pose_mtx);
                // check ptr_pose validity (it cannot be strange value)
                bool is_valid = true;
                // TODO: can add more validity check (for example, the ptr_pose should be in front of the camera)
                for (int i = 0; i < 3; i++)
                {
                    if (ptr_pose(i) != ptr_pose(i))
                    {
                        is_valid = false;
                    }
                }
                
                //cout << "I triangulate: " << ptr_pose << endl;
                
                /* 2. calculate r and H */
                if (is_valid)
                {
                    num_measure++;
                    // construct H matrix use ptr_pose, item.second.start_frame and current_frame
                    VectorXd ri;
                    MatrixXd Hi;
                    getResidualH(ri, Hi, ptr_pose, measure_mtx, pose_mtx, item.second.start_frame);
                    row_H += (2 * num_frame - 3); // after feature error marginalization
                    
                    // TODO: outlier reject: Chi-square test
                    
                    residual_list.push_back(ri);
                    H_mtx_list.push_back(Hi);
                    H_mtx_block_size_list.push_back(2 * num_frame - 3);
                    
                    item.second.is_used = true;
                }
            }
            else
            {
                // not enough number of frame, does not generate measure
                item.second.is_used = true;
            }
        }
    }
    
    if (num_measure == 0) // this may due to hovering
    {
        
    }
    else
    {
        int col_H = (int)fullErrorCovariance.rows();;
        // use ri and Hi to do KF update
        /* 1. construct H matrix */
        MatrixXd H = MatrixXd::Zero(row_H, col_H);
        VectorXd r = VectorXd::Zero(row_H);
        
        std::list<MatrixXd>::iterator itr_H = H_mtx_list.begin();
        std::list<VectorXd>::iterator itr_r = residual_list.begin();
        std::list<int>::iterator itr_H_size = H_mtx_block_size_list.begin();
        
        int row_H_count = 0;
        for (int i = 0; i < num_measure; i++)
        {
//            cout << (*itr_H).cols() << ", " << (*itr_H).rows() << endl;
//            cout << *itr_H_size << endl;
            
            H.block(row_H_count, 0, *itr_H_size, col_H) = (*itr_H);
            r.segment(row_H_count, *itr_H_size) = (*itr_r);
            row_H_count += *itr_H_size;
            
            itr_H++;
            itr_r++;
            itr_H_size++;
        }

        VectorXd delta_x;
        // when there are a lot of features, use QR of H to speed up computation
        if (H.rows()>H.cols())
        {
            HouseholderQR<MatrixXd> qr(H.cast<double>());
            MatrixXd R = qr.matrixQR().triangularView<Upper>();
            MatrixXd Q = qr.householderQ();
            MatrixXd Th = R.topRows(col_H).cast<double>();
            MatrixXd Q1 = Q.leftCols(col_H).cast<double>();
            
            /* 3. calculate Kalman gain */
            MatrixXd Rq = MatrixXd::Identity(row_H, row_H) * measure_noise*measure_noise;
            MatrixXd tmpK = (Th * fullErrorCovariance * Th.transpose() + Rq).inverse();
            MatrixXd K = fullErrorCovariance * Th.transpose() * tmpK;
        
            /* 4. update error covariance */
            MatrixXd ImKH = MatrixXd::Identity(col_H, col_H) - K * Th;
            fullErrorCovariance = ImKH * fullErrorCovariance * ImKH.transpose() + K * Rq * K.transpose();
        
            delta_x = K * Q1.transpose() * r;
        }
        else
        {
            MatrixXd Rq = MatrixXd::Identity(row_H, row_H) * measure_noise*measure_noise;
            MatrixXd tmpK = (H * fullErrorCovariance * H.transpose() + Rq).inverse();
            MatrixXd K = fullErrorCovariance * H.transpose() * tmpK;
            MatrixXd ImKH = MatrixXd::Identity(col_H, col_H) - K * H;
            fullErrorCovariance = ImKH * fullErrorCovariance*ImKH.transpose() + K * Rq * K.transpose();
            delta_x = K * r;
        }
        
        
        correctNominalState(delta_x);

    }
    
    
    // remove all is_used == true sliding state
    int feature_count;
    int used_count;
    list<int> frame_to_remove;
    frame_to_remove.clear();
    for (int i = 0; i < current_frame; i++)
    {
        feature_count = 0;
        used_count = 0;
        for (auto & item : feature_record_dict)
        {
            if (item.second.start_frame <= i)
            {
                feature_count++;
            }
            if (item.second.is_used == true)
            {
                used_count++;
            }
        }
        if (feature_count == used_count) // all features under this state are used
        {
            if (feature_count > 0)
            {
                frame_to_remove.push_back(i);
            }
        }
        
    }
    
//    cout << __FILE__ << ":" << __LINE__ <<endl;
//    printNominalState(true);
    
    int offset = 0;
    for (auto & frame : frame_to_remove)  // frame is ordered
    {
        //cout << frame << endl;
        //printNominalState(true);
        
        removeSlideState(frame - offset, current_frame + 1);
        removeFrameFeatures(frame - offset);
        current_frame--;
        
        //printNominalState(true);
        offset++;
    }
    
//    cout << __FILE__ << ":" << __LINE__ <<endl;
//    printNominalState(true);
    
    return;
}

void MSCKF::addSlideState()
{
    SlideState newState;
    int nominalStateLength = (int)fullNominalState.size();
    int errorStateLength = (int)fullErrorCovariance.rows();
    
    VectorXd tmpNominal = VectorXd::Zero(nominalStateLength + NOMINAL_POSE_STATE_SIZE);
    MatrixXd tmpCovariance = MatrixXd::Zero(errorStateLength + ERROR_POSE_STATE_SIZE, errorStateLength + ERROR_POSE_STATE_SIZE);
    MatrixXd Jpi = MatrixXd::Zero(9, errorStateLength);
    
    tmpNominal.head(nominalStateLength) = fullNominalState;
    tmpNominal.segment(nominalStateLength, NOMINAL_POSE_STATE_SIZE) = fullNominalState.head(NOMINAL_POSE_STATE_SIZE);
    fullNominalState = tmpNominal;
    
    newState.q = fullNominalState.head(4);
    newState.p = fullNominalState.segment(4, 3);
    newState.v = fullNominalState.segment(7, 3);
    
    slidingWindow.push_back(newState);
    
    Jpi.block<3,3>(0, 0) = Matrix3d::Identity(3, 3);
    Jpi.block<3,3>(3, 3) = Matrix3d::Identity(3, 3);
    Jpi.block<3,3>(6, 6) = Matrix3d::Identity(3, 3);
    tmpCovariance.block(0, 0, errorStateLength, errorStateLength) = fullErrorCovariance;
    tmpCovariance.block(errorStateLength, 0, 9, errorStateLength) = Jpi * fullErrorCovariance;
    tmpCovariance.block(0, errorStateLength, errorStateLength, 9) = fullErrorCovariance * Jpi.transpose();
    tmpCovariance.block<ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE>(errorStateLength, errorStateLength) =
        Jpi * fullErrorCovariance * Jpi.transpose();
    
    fullErrorCovariance = tmpCovariance;
}

void MSCKF::addFeatures(const vector<pair<int, Vector3d>> &image)
{
    // add features to the feature record
    for (auto & id_pts : image)
    {
        int   id = id_pts.first;
        double x = id_pts.second(0);
        double y = id_pts.second(1);
        double z = id_pts.second(2);
        
        // this is a new feature record
        if (feature_record_dict.find(id) == feature_record_dict.end())
        {
            feature_record_dict[id] = FeatureRecord(current_frame, Vector3d(x, y, z));
        }
        else // append to existing record
        {
            feature_record_dict[id].feature_points.push_back(FeatureInformation(Vector3d(x, y, z)));
            feature_record_dict[id].is_lost = false;
        }
    }
    
}


// CAUTION: make sure this function is not called at wrong time, and make sure that the index is valid
//          this function does not check index validity yet
void MSCKF::removeSlideState(int index, int total)
{
    //TODO: check total with (nominalStateLength-NOMINAL_POSE_STATE_SIZE-3)/NOMINAL_POSE_STATE_SIZE
    
    int nominalStateLength = (int)fullNominalState.size();
    int errorStateLength = (int)fullErrorCovariance.rows();
    
    VectorXd tmpNominal = VectorXd::Zero(nominalStateLength - NOMINAL_POSE_STATE_SIZE);
    MatrixXd tmpCovariance = MatrixXd::Zero(errorStateLength - ERROR_POSE_STATE_SIZE, errorStateLength - ERROR_POSE_STATE_SIZE);
    
    /* remove nomial state */
    tmpNominal.head(NOMINAL_STATE_SIZE + 3) = fullNominalState.head(NOMINAL_STATE_SIZE + 3);
    
    for (int i = 0; i < total; i++)
    {
        if (i == index)
        {
            continue;
        }
        else if (i < index)
        {
            tmpNominal.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE) =
                fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE);
        }
        else
        {
            tmpNominal.segment(NOMINAL_STATE_SIZE+3 + (i-1)*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE) =
                fullNominalState.segment(NOMINAL_STATE_SIZE+3 + i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE);
        }
    }
    
    fullNominalState = tmpNominal;
    
    /* remove error covariance */
    int position_index; // the position of state with index in the matrix
    position_index = ERROR_STATE_SIZE + 3 + index * ERROR_POSE_STATE_SIZE;
    
    tmpCovariance.block(0, 0, position_index, position_index) =
        fullErrorCovariance.block(0, 0, position_index, position_index);
    
    
    tmpCovariance.block(0, position_index,
                        position_index,
                        errorStateLength - position_index - ERROR_POSE_STATE_SIZE) =
        fullErrorCovariance.block(0, position_index + ERROR_POSE_STATE_SIZE,
                                  position_index,
                                  errorStateLength - position_index - ERROR_POSE_STATE_SIZE);
    
    tmpCovariance.block(position_index,0,
                        errorStateLength - position_index - ERROR_POSE_STATE_SIZE,
                        position_index) =
        fullErrorCovariance.block(position_index + ERROR_POSE_STATE_SIZE,0,
                                  errorStateLength - position_index - ERROR_POSE_STATE_SIZE,
                                  position_index);
    
    tmpCovariance.block(position_index,position_index,
                        errorStateLength - position_index - ERROR_POSE_STATE_SIZE,
                        errorStateLength - position_index - ERROR_POSE_STATE_SIZE) =
        fullErrorCovariance.block(position_index + ERROR_POSE_STATE_SIZE, position_index + ERROR_POSE_STATE_SIZE,
                                  errorStateLength - position_index - ERROR_POSE_STATE_SIZE,
                                  errorStateLength - position_index - ERROR_POSE_STATE_SIZE);
    
    fullErrorCovariance = tmpCovariance;
    
    /* remove sliding state */
    std::list<SlideState>::iterator itr;
    itr = slidingWindow.begin();
    for (int idx = 0; idx < index; idx++)
    {
        itr++;
    }
    slidingWindow.erase(itr);
}

void MSCKF::removeFrameFeatures(int index)
{
    list<int> id_to_remove;
    id_to_remove.clear();
    for (auto & item : feature_record_dict)
    {
        if (item.second.start_frame < index)
        {
            item.second.feature_points.erase(
                item.second.feature_points.begin() + (index - item.second.start_frame)
            );
        }
        else if (item.second.start_frame == index)
        {
            item.second.feature_points.erase(item.second.feature_points.begin());
        }
        else
        {
            item.second.start_frame = item.second.start_frame - 1;
        }
        
        if (item.second.feature_points.size() == 0)
        {
            id_to_remove.push_back(item.first);
        }
    }
    // remove feature record with 0 feature information
    for (auto &id : id_to_remove)
    {
        feature_record_dict.erase(id);
    }
}

Vector2d MSCKF::projectWorldPoint(Vector3d ptr)
{
    return cam.h(ptr);
}

Vector2d MSCKF::projectPoint(Vector3d feature_pose, Matrix3d R_gb, Vector3d p_gb, Vector3d p_cb)
{
    Vector2d zij;
    zij = cam.h(R_cb * R_gb.transpose() * (feature_pose - p_gb) + p_cb);
    
    return zij;
}

/*
 *   frame_offset: used to place HxBj in right place in H
 */
void MSCKF::getResidualH(VectorXd& ri, MatrixXd& Hi, Vector3d feature_pose, MatrixXd measure, MatrixXd pose_mtx, int frame_offset)
{
    int num_frame = (int)pose_mtx.cols();
    int errorStateLength = (int)fullErrorCovariance.rows();
    
    ri = VectorXd::Zero(2 * num_frame);
    Hi = MatrixXd::Zero(2 * num_frame, errorStateLength);    // Hi cols == error state length
    
    MatrixXd HxBj, Hc;
    MatrixXd Mij, tmp39;
    
    MatrixXd Hfi;
    MatrixXd Hf; // use double precision to increase numerial result
    Hfi = MatrixXd::Zero(2 * num_frame, 3);
    
    for(int j = 0; j < num_frame; j++)
    {
//        cout << "frame is " << frame_offset+j << endl;
        Matrix3d R_gb = quaternion_to_R(pose_mtx.block<4, 1>(0, j));
        Vector3d feature_in_c = R_cb * R_gb.transpose() * (feature_pose - pose_mtx.block<3, 1>(4, j)) + fullNominalState.segment(16, 3);
        Vector2d projPtr = projectPoint(feature_pose, R_gb, pose_mtx.block<3, 1>(4, j), fullNominalState.segment(16, 3));
        
//        cout << "measure is " << measure.col(j).transpose() << endl;
//        cout << "estimat is " << projPtr << endl;
        ri.segment(j * 2, 2) = measure.col(j) - projPtr;
        
        Mij = cam.Jh(feature_in_c) * R_cb * R_gb.transpose();
        tmp39 = MatrixXd::Zero(3, 9);
        tmp39.block<3, 3>(0, 0) = skew_mtx(feature_pose - pose_mtx.block<3, 1>(4, j));
        tmp39.block<3, 3>(0, 3) = -Matrix3d::Identity();
        
        HxBj = Mij * tmp39;                           // 2x9
        Hc = cam.Jh(feature_in_c);   // 2x3
//        cout << "HxBj is " << HxBj;
//        cout << "Hc is " << Hc;
        
        Hi.block<2, 9>(j * 2, ERROR_STATE_SIZE + 3 + ERROR_POSE_STATE_SIZE * (frame_offset + j)) = HxBj;
        Hi.block<2, 3>(j * 2, ERROR_STATE_SIZE) = Hc;
        
        Hf = cam.Jh(feature_pose) * R_cb * R_gb.transpose();
        Hfi.block<2, 3>(j * 2, 0) = Hf.cast<double>();
    }
    // now carry out feature error marginalization
    JacobiSVD<MatrixXd> svd(Hfi.transpose(), ComputeFullV);
    MatrixXd left_null = svd.matrixV().cast<double>().rightCols(2 * num_frame - 3).transpose();
    
//    MatrixXd S = svd.singularValues().asDiagonal();
//    MatrixXd U = svd.matrixU();
//    MatrixXd V = svd.matrixV();
//    MatrixXd D = Hfi.transpose()-U*S*V.transpose();
//    std::cout << "\n" << D.norm() << "  " << sqrt((D.adjoint()*D).trace()) << "\n";
    
//    printf("left null size (%d, %d)\n", left_null.rows(), left_null.cols());
//    printf("ri size (%d, %d)\n", ri.rows(), ri.cols());
//    printf("Hi size (%d, %d)\n", Hi.rows(), Hi.cols());
    
    ri = left_null * ri;
    Hi = left_null * Hi;
    
//    cout << "one measure, one H" << endl;
//    cout << "------------------" << endl;
//    cout << ri << endl;
//    cout << Hi << endl;
//    printf("ri size (%d, %d)\n", ri.rows(), ri.cols());
//    printf("Hi size (%d, %d)\n", Hi.rows(), Hi.cols());
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

Vector4d MSCKF::getQuaternion()
{
    return fullNominalState.head(4);
}

Matrix3d MSCKF::getRotation()
{
    Vector4d q = fullNominalState.head(4);
    return quaternion_to_R(q);
}

Vector3d MSCKF::getPosition()
{
    return fullNominalState.segment(4, 3);
}

Vector3d MSCKF::getVelocity()
{
    return fullNominalState.segment(7, 3);
}
Vector3d MSCKF::getGyroBias()
{
    return fullNominalState.segment(10, 3);
}
Vector3d MSCKF::getAcceBias()
{
    return fullNominalState.segment(13, 3);
}
Vector3d MSCKF::getVIOffset()
{
    return fullNominalState.segment(16, 3);
}
