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

static Vector3f g(0.0f, 0.0f, -9.81f);

MSCKF::MSCKF()
{
    //slidingWindow = new VectorXf[SLIDING_WINDOW_SIZE];
    //slidingWindow[0] = VectorXf(BODY_POSE_STATE_SIZE);
    
    nominalState = VectorXf::Zero(NOMINAL_STATE_SIZE);
    //nominalState.segment(0, 4) = Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
    errorState = VectorXf::Zero(ERROR_STATE_SIZE);
    extrinsicP = VectorXf::Zero(3);
    
    fullNominalState = VectorXf::Zero(NOMINAL_STATE_SIZE+3);
    fullErrorState = VectorXf::Zero(ERROR_STATE_SIZE+3);
    
    phi = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    errorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    fullErrorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE+3, ERROR_STATE_SIZE+3);
    fullErrorCovariance.block<3,3>(ERROR_STATE_SIZE,ERROR_STATE_SIZE) = Matrix3f::Identity();
    
    Nc = MatrixXf::Zero(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    setNoiseMatrix(0.1f, 0.1f, 0.1f, 0.1f);
    
    current_time = -1.0f;
    
    cam.setImageSize(480, 752);
    
    cam.setIntrinsicMtx(365.07984, 365.12127, 381.0196, 254.4431);
    cam.setDistortionParam(-2.842958e-1,
                           8.7155025e-2,
                           -1.4602925e-4,
                           -6.149638e-4,
                           -1.218237e-2);
    
    current_frame = 0;
    
    R_cb <<
        0, -1, 0,
        0,  0, 1,
       -1,  0, 0;
//    Ric <<
//    0, 0, -1,
//    -1, 0, 0,
//    0, 1, 0;
//    Tic << 0.0, -0.14, 0.02;
}

MSCKF::~MSCKF()
{
    //delete[] slidingWindow;
}

void MSCKF::resetError()
{
    int errorStateLength = (int)fullErrorState.size();
    
    errorState = VectorXf::Zero(ERROR_STATE_SIZE);
    //errorCovariance = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
    
    // CAUTION: their sizes are not fixed
    fullErrorState = VectorXf::Zero(errorStateLength);
    //fullErrorCovariance = MatrixXf::Identity(errorStateLength, errorStateLength);
    phi = MatrixXf::Identity(ERROR_STATE_SIZE,ERROR_STATE_SIZE);
}

void MSCKF::setNoiseMatrix(float dgc, float dac, float dwgc, float dwac)
{
    Nc.block<3,3>(0, 0) = Matrix3f::Identity()*dgc;
    Nc.block<3,3>(6, 6) = Matrix3f::Identity()*dac;
    Nc.block<3,3>(9, 9) = Matrix3f::Identity()*dwgc;
    Nc.block<3,3>(12, 12) = Matrix3f::Identity()*dwac;
}

void MSCKF::setMeasureNoise(float _noise)
{
    measure_noise = _noise;
}

void MSCKF::setNominalState(Vector4f q, Vector3f p, Vector3f v, Vector3f bg, Vector3f ba)
{
    fullNominalState.segment(0, 4) = q;
    fullNominalState.segment(4, 3) = p;
    fullNominalState.segment(7, 3) = v;
    fullNominalState.segment(10, 3) = bg;
    fullNominalState.segment(13, 3) = ba;
    
    nominalState = fullNominalState.head(NOMINAL_STATE_SIZE);
}

void MSCKF::setCalibParam(Vector3f p_cb, float fx, float fy, float ox, float oy, float k1, float k2, float p1, float p2, float k3)
{
    fullNominalState.segment(16, 3) = p_cb;
    cam.setIntrinsicMtx(fx, fy, ox, oy);
    cam.setDistortionParam(k1, k2, p1, p2, k3);
    
}

void MSCKF::setIMUCameraRotation(Matrix3f _R_cb)
{
    R_cb = _R_cb;
}

void MSCKF::correctNominalState(VectorXf delta)
{
    fullNominalState.segment(0, 4) = quaternion_correct(fullNominalState.segment(0, 4), delta.segment(0, 3));
    fullNominalState.segment(4, 3) = fullNominalState.segment(4, 3)+delta.segment(3, 3);
    fullNominalState.segment(7, 3) = fullNominalState.segment(7, 3)+delta.segment(6, 3);
    fullNominalState.segment(10, 3) = fullNominalState.segment(10, 3)+delta.segment(9, 3);
    fullNominalState.segment(13, 3) = fullNominalState.segment(13, 3)+delta.segment(12, 3);
    fullNominalState.segment(16, 3) = fullNominalState.segment(16, 3)+delta.segment(15, 3);   //p_cb
    
    // loop to correct sliding states
    std::list<SlideState>::iterator itr = slidingWindow.begin();
    for (int i=0; i<=current_frame;i++)
    {
        fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, 4) =
            quaternion_correct(fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, 4),
                               delta.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE, 3));
        itr->q = fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, 4);
        
        fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE+4, 3) =
            fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE+4, 3) +
        delta.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE+3, 3);
        itr->p = fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE+4, 3);
        
        fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE+7, 3) =
            fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE+7, 3) +
            delta.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE+7, 3);
        itr->v = fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE+7, 3);
        
        itr++;
    }
    
    nominalState = fullNominalState.head(NOMINAL_STATE_SIZE);
}

void MSCKF::processIMU(float t, Vector3f linear_acceleration, Vector3f angular_velocity)
{
    Vector4f small_rotation;
    Matrix3f d_R, prev_R, average_R, phi_vbg;
    Vector3f s_hat, y_hat;
    Vector3f tmp_vel, tmp_pos;
    // read nomial state to get variables
    spatial_quaternion = nominalState.segment(0, 4);
    spatial_position = nominalState.segment(4, 3);
    spatial_velocity = nominalState.segment(7, 3);
    gyro_bias = nominalState.segment(10, 3);
    acce_bias = nominalState.segment(13, 3);
    
    spatial_rotation = quaternion_to_R(spatial_quaternion);
    std::cout << spatial_rotation << std::endl;
    if (current_time < 0.0f)
    {
        current_time = t;
        prev_w = angular_velocity - gyro_bias;
        prev_a = linear_acceleration - acce_bias;
        return;
    }
    
    float dt = t - current_time;
    
    current_time = t;
    curr_w = angular_velocity - gyro_bias;
    curr_a = linear_acceleration - acce_bias;
    
    //calculate q_B{l+1}B{l}
    small_rotation = delta_quaternion(prev_w, curr_w, dt);
    d_R = quaternion_to_R(small_rotation);
    
    // defined in paper P.49
    s_hat = 0.5f * dt * (d_R.transpose()*curr_a + prev_a);
    y_hat = 0.5f * dt * s_hat;
    
    /* update nominal state */
    tmp_vel = spatial_velocity + spatial_rotation.transpose()*s_hat + g*dt;
    tmp_pos = spatial_position + spatial_velocity*dt
                               + spatial_rotation.transpose()*y_hat + 0.5f*g*dt*dt;
    
    spatial_velocity = tmp_vel;
    spatial_position = tmp_pos;
    prev_R = spatial_rotation;
    spatial_rotation = d_R*spatial_rotation;
    
    /* propogate error covariance */
    average_R = prev_R.transpose()+spatial_rotation.transpose();
    //1. phi_pq
    phi.block<3,3>(3,0) = -skew_mtx(prev_R*y_hat);
    //2. phi_vq
    phi.block<3,3>(6,0) = -skew_mtx(prev_R*s_hat);
    //3. one bloack need to times dt;
    phi.block<3,3>(3,6) = Matrix3f::Identity()*dt;
    //4. phi_qbg
    phi.block<3,3>(0,9) = -0.5f*dt*average_R;
    //5. phi_vbg
    phi_vbg = 0.25f*dt*dt*(skew_mtx(spatial_rotation.transpose()*curr_a)*average_R);
    phi.block<3,3>(6,9) = phi_vbg;
    //6. phi_pbg
    phi.block<3,3>(3,9) = 0.5f*dt*phi_vbg;
    
    //7. phi_vba
    phi.block<3,3>(6,12) = -0.5f*dt*average_R;
    //8. phi_pba
    phi.block<3,3>(3,12) = -0.25f*dt*dt*average_R;
    
    std::cout << "phi is" << std::endl;
    std::cout << phi << std::endl;
    
    errorCovariance = phi*(errorCovariance+0.5*dt*Nc)*phi.transpose() + Nc;
    
    fullErrorCovariance.block<ERROR_STATE_SIZE, ERROR_STATE_SIZE>(0, 0) = errorCovariance;
    
    return;
}

void MSCKF::processImage(const vector<pair<int, Vector3d>> &image, vector<pair<Vector3d, Vector3d>> &corres)
{
    printf("input feature: %lu", image.size());
    
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
    if (current_frame == SLIDING_WINDOW_SIZE)
    {
        removeSlideState(1, SLIDING_WINDOW_SIZE);
        removeFrameFeatures(1);
        current_frame--;
    }
    
    addSlideState();
    addFeatures(image);     // is_lost modified here

    //check is_lost to get measurement
    MatrixXf measure_mtx;
    MatrixXf pose_mtx;
    Vector3f ptr_pose;
    
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
            if (current_frame-item.second.start_frame>=3)
            {
                /* 1. prepare to do triangulation */
                std::vector<FeatureInformation>::iterator itr_f = item.second.feature_points.begin();
                std::list<SlideState>::iterator           itr_s = slidingWindow.begin();
                for (int i = 0; i < item.second.start_frame; i++)
                {
                    itr_s ++;
                }
                
                int num_frame = current_frame-item.second.start_frame;
                
                measure_mtx = MatrixXf::Zero(2,num_frame);
                pose_mtx = MatrixXf::Zero(7,num_frame);
                
                for (int i = item.second.start_frame; i < current_frame; i++)
                {
                    // construct measure
                    measure_mtx(0,i-item.second.start_frame) = itr_f->point.x();
                    measure_mtx(1,i-item.second.start_frame) = itr_f->point.y();
                
                    // construct pose
                    pose_mtx.block<4,1>(0,i-item.second.start_frame) = itr_s->q;
                    pose_mtx.block<3,1>(4,i-item.second.start_frame) = itr_s->p;
                    
                    itr_f++;
                    itr_s++;
                }
                
                ptr_pose = cam.triangulate(measure_mtx, pose_mtx);
                // check ptr_pose validity (it cannot be strange value)
                bool is_valid = true;
                // TODO: can add more validity check (for example, the ptr_pose should be in front of the camera)
                for (int i=0; i<3;i++)
                {
                    if (ptr_pose(i)!=ptr_pose(i))
                        is_valid = false;
                }
                
                /* 2. calculate r and H */
                if (is_valid)
                {
                    num_measure++;
                    row_H += (2*num_frame - 3); // after feature error marginalization
                    // construct H matrix use ptr_pose, item.second.start_frame and current_frame
                    VectorXf ri;
                    MatrixXf Hi;
                    getResidualH(ri, Hi, ptr_pose, measure_mtx, pose_mtx, item.second.start_frame);
                    
                    // TODO: outlier reject: Chi-square test
                    
                    residual_list.push_back(ri);
                    H_mtx_list.push_back(Hi);
                    H_mtx_block_size_list.push_back(2*num_frame - 3);
                    
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
        int col_H = (int)fullErrorState.size();
        // use ri and Hi to do KF update
        /* 1. construct H matrix */
        MatrixXd H = MatrixXd::Zero(row_H, col_H);
        VectorXf r = VectorXf::Zero(row_H);
        std::list<MatrixXf>::iterator itr_H = H_mtx_list.begin();
        std::list<VectorXf>::iterator itr_r = residual_list.begin();
        std::list<int>::iterator itr_H_size = H_mtx_block_size_list.begin();
        int row_H_count = 0;
        for (int i = 0; i < num_measure; i++)
        {
            H.block(row_H_count, 0, *itr_H_size, col_H) = (*itr_H).cast<double>();
            r.segment(row_H_count, *itr_H_size) = (*itr_r);
            row_H_count += *itr_H_size;
            
            itr_H++;
            itr_r++;
            itr_H_size++;
        }
        
        /* 2. decompose H */
        HouseholderQR<MatrixXd> qr(H);
        MatrixXd R = qr.matrixQR().triangularView<Upper>();
        MatrixXd Q = qr.householderQ();
        
        MatrixXf Th = R.topRows(col_H).cast<float>();
        MatrixXf Q1 = Q.leftCols(col_H).cast<float>();
        
        /* 3. calculate Kalman gain */
        MatrixXf Rq = MatrixXf::Identity(col_H, col_H) * measure_noise*measure_noise;
        MatrixXf tmpK = (Th*fullErrorCovariance*Th.transpose() + Rq).inverse();
        MatrixXf K = fullErrorCovariance*Th.transpose()*tmpK;
        
        /* 4. update error covariance */
        fullErrorCovariance = (MatrixXf::Identity(col_H, col_H) - K*Th)*fullErrorCovariance;
        
        VectorXf delta_x = K*Q1.transpose()*r;
        correctNominalState(delta_x);
    }
    
    
    // remove all is_used == true sliding state
    int feature_count;
    int used_count;
    list<int> frame_to_remove;
    frame_to_remove.clear();
    for (int i=0; i<=current_frame;i++)
    {
        feature_count = 0;
        used_count = 0;
        for (auto & item : feature_record_dict)
        {
            if (item.second.start_frame < i)
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
            frame_to_remove.push_back(i);
        }
        
    }
    int offset = 0;
    for (auto &frame:frame_to_remove)  // frame is ordered
    {
        removeSlideState(frame-offset, current_frame+1);
        removeFrameFeatures(frame-offset);
        current_frame--;
        offset++;
    }
    
    
    // move to next frame by the end
    current_frame++;
}

void MSCKF::addSlideState()
{
    SlideState newState;
    int nominalStateLength = (int)fullNominalState.size();
    int errorStateLength = (int)fullErrorState.size();
    
    VectorXf tmpNominal = VectorXf::Zero(nominalStateLength+NOMINAL_POSE_STATE_SIZE);
    VectorXf tmpError   = VectorXf::Zero(errorStateLength+ERROR_POSE_STATE_SIZE);
    MatrixXf tmpCovariance = MatrixXf::Zero(errorStateLength+ERROR_POSE_STATE_SIZE,errorStateLength+ERROR_POSE_STATE_SIZE);
    MatrixXf Jpi = MatrixXf::Zero(9, errorStateLength);
    
    tmpNominal.head(nominalStateLength) = fullNominalState;
    tmpNominal.segment(nominalStateLength,NOMINAL_POSE_STATE_SIZE) = fullNominalState.head(NOMINAL_POSE_STATE_SIZE);
    fullNominalState = tmpNominal;
    
    tmpError.head(errorStateLength) = fullErrorState;
    tmpError.segment(errorStateLength,ERROR_POSE_STATE_SIZE) = fullErrorState.head(ERROR_POSE_STATE_SIZE);
    fullErrorState = tmpError;
    
    newState.q = fullNominalState.head(4);
    newState.p = fullNominalState.segment(4,3);
    newState.v = fullNominalState.segment(7,3);
    
    slidingWindow.push_back(newState);
    
    Jpi.block<3,3>(0,0) = Matrix3f::Identity(3,3);
    Jpi.block<3,3>(3,3) = Matrix3f::Identity(3,3);
    Jpi.block<3,3>(6,6) = Matrix3f::Identity(3,3);
    tmpCovariance.block(0,0, errorStateLength, errorStateLength) = fullErrorCovariance;
    tmpCovariance.block(errorStateLength,0,9,errorStateLength) = Jpi*fullErrorCovariance;
    tmpCovariance.block(0,errorStateLength,errorStateLength,9) = fullErrorCovariance*Jpi.transpose();
    tmpCovariance.block<ERROR_POSE_STATE_SIZE,ERROR_POSE_STATE_SIZE>(errorStateLength,errorStateLength) =
        Jpi*fullErrorCovariance*Jpi.transpose();
    
    fullErrorCovariance = tmpCovariance;
}

void MSCKF::addFeatures(const vector<pair<int, Vector3d>> &image)
{
    // add features to the feature record
    for (auto & id_pts : image)
    {
        int   id = id_pts.first;
        float x = id_pts.second(0);
        float y = id_pts.second(1);
        float z = id_pts.second(2);
        
        // this is a new feature record
        if (feature_record_dict.find(id) == feature_record_dict.end())
        {
            feature_record_dict[id] = FeatureRecord(current_frame, Vector3f(x, y, z));
        }
        else // append to existing record
        {
            feature_record_dict[id].feature_points.push_back(FeatureInformation(Vector3f(x, y, z)));
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
    int errorStateLength = (int)fullErrorState.size();
    
    VectorXf tmpNominal = VectorXf::Zero(nominalStateLength-NOMINAL_POSE_STATE_SIZE);
    VectorXf tmpError   = VectorXf::Zero(errorStateLength-ERROR_POSE_STATE_SIZE);
    MatrixXf tmpCovariance = MatrixXf::Zero(errorStateLength-ERROR_POSE_STATE_SIZE,errorStateLength-ERROR_POSE_STATE_SIZE);
    
    tmpNominal.head(NOMINAL_STATE_SIZE+3) = fullNominalState.head(NOMINAL_STATE_SIZE+3);
    tmpError.head(ERROR_STATE_SIZE+3) = fullErrorState.head(ERROR_STATE_SIZE+3);
    
    for (int i=0; i<total;i++)
    {
        if (i == index)
        {
            continue;
        }
        else if (i < index)
        {
            tmpNominal.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE) =
                fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE);
            tmpError.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE) =
                fullErrorState.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE);
        }
        else
        {
            tmpNominal.segment(NOMINAL_STATE_SIZE+3+(i-1)*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE) =
                fullNominalState.segment(NOMINAL_STATE_SIZE+3+i*NOMINAL_POSE_STATE_SIZE, NOMINAL_POSE_STATE_SIZE);
            tmpError.segment(ERROR_STATE_SIZE+3+(i-1)*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE) =
                fullErrorState.segment(ERROR_STATE_SIZE+3+i*ERROR_POSE_STATE_SIZE, ERROR_POSE_STATE_SIZE);
        }
    }
    
    fullNominalState = tmpNominal;
    fullErrorState = tmpError;
    
    int position_index; // the position of state with index in the matrix
    position_index = ERROR_STATE_SIZE + 3 + index * ERROR_POSE_STATE_SIZE;
    
    tmpCovariance.block(0, 0, position_index, position_index) =
        fullErrorCovariance.block(0, 0, position_index, position_index);
    
    
    tmpCovariance.block(0, position_index,
                        position_index,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE) =
        fullErrorCovariance.block(0, position_index+ERROR_POSE_STATE_SIZE,
                                  position_index,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE);
    
    tmpCovariance.block(position_index,0,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                        position_index) =
        fullErrorCovariance.block(position_index+ERROR_POSE_STATE_SIZE,0,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                                  position_index);
    
    tmpCovariance.block(position_index,position_index,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                        errorStateLength-position_index-ERROR_POSE_STATE_SIZE) =
        fullErrorCovariance.block(position_index+ERROR_POSE_STATE_SIZE,position_index+ERROR_POSE_STATE_SIZE,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE,
                                  errorStateLength-position_index-ERROR_POSE_STATE_SIZE);
    fullErrorCovariance = tmpCovariance;
    
    std::list<SlideState>::iterator itr;
    itr = slidingWindow.begin();
    for (int idx = 0; idx<index;idx++)
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
                item.second.feature_points.begin()+(index-item.second.start_frame)
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

Vector2f MSCKF::projectPoint(Vector3f feature_pose, Matrix3f R_gb, Vector3f p_gb, Vector3f p_cb)
{
    Vector2f zij;
    zij = cam.h(R_cb*R_gb.transpose()*(feature_pose - p_gb)+p_cb);
    
    return zij;
}

void MSCKF::getResidualH(VectorXf& ri, MatrixXf& Hi, Vector3f feature_pose, MatrixXf measure, MatrixXf pose_mtx, int frame_offset)
{
    int num_frame = (int)pose_mtx.cols();
    int errorStateLength = (int)fullErrorState.size();
    
    ri = VectorXf::Zero(2*num_frame);
    Hi = MatrixXf::Zero(2*num_frame, errorStateLength);    // Hi cols == error state length
    
    MatrixXf HxBj, Hc;
    MatrixXf Mij, tmp39;
    
    MatrixXd Hfi;
    MatrixXf Hf; // use double precision to increase numerial result
    Hfi = MatrixXd::Zero(2*num_frame, 3);
    
    Hc = cam.Jh(fullNominalState.segment(16, 3));   // 2x3
    for(int j =0; j < num_frame; j++)
    {
        Matrix3f R_gb = quaternion_to_R(pose_mtx.block<4,1>(0,j));
        
        ri.segment(j*2, 2) = measure.col(j) - projectPoint(feature_pose, R_gb, pose_mtx.block<3,1>(4,j), fullNominalState.segment(16, 3));
        
        Mij = cam.Jh(R_gb.transpose()*feature_pose)*R_cb*R_gb.transpose();
        tmp39 = MatrixXf::Zero(3,9);
        tmp39.block<3,3>(0,0) = skew_mtx(feature_pose-pose_mtx.block<3,1>(4,j));
        tmp39.block<3,3>(0,3) = -Matrix3f::Identity();
        
        HxBj = Mij*tmp39;                           // 2x9
        
        Hi.block<2,9>(j*2,ERROR_STATE_SIZE+3+ERROR_POSE_STATE_SIZE*(frame_offset+j)) = HxBj;
        Hi.block<2,3>(j*2,ERROR_STATE_SIZE) = Hc;
        
        Hf = cam.Jh(feature_pose)*R_cb*R_gb.transpose();
        Hfi.block<2,3>(j*2,0) = Hf.cast<double>();
    }
    // now carry out feature error marginalization
    JacobiSVD<MatrixXd> svd(Hfi.transpose(), Eigen::ComputeFullV);
    MatrixXf left_null = svd.matrixV().cast<float>().transpose();
    
    ri = left_null*ri;
    Hi = left_null*Hi;
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

void MSCKF::printErrorState(bool is_full)
{
    int errorStateLength = (int)fullErrorState.size();
    
    if (is_full == true)
    {
        std::cout<<"full error state is"<<std::endl;
        for (int i = 0; i < errorStateLength; i++)
        {
            std::cout<< fullErrorState(i);
            if (i!=ERROR_STATE_SIZE-1)
                std::cout<<" ";
            else
                std::cout<<"|";
        }
        std::cout<<std::endl;
    }
    else
    {
        std::cout<<"error state is"<<std::endl;
        for (int i = 0; i < ERROR_STATE_SIZE; i++)
        {
            std::cout<< fullErrorState(i) << " ";
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
    