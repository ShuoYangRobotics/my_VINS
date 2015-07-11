#include "data_generator.h"
#include "camera.h"
#include "calib.h"
#include "odometry.h"

#include <gtest/gtest.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

Calib* calib;
DataGenerator* generator;
MSCKF* msckf;
CameraMeasurements* cameraMeasurements;

void setup()
{
    calib = new Calib();
    calib->camera.setIntrinsicParam(365.07984, 365.12127, 381.01962, 254.44318);
    calib->camera.setDistortionParam(-2.842958e-1, 8.7155025e-2, -1.4602925e-4, -6.149638e-4, -1.218237e-2);

    calib->delta_t = (double) 1 / DataGenerator::FREQ;
    calib->CI_q = Quaterniond((Matrix3d() << 0, -1, 0, 0, 0, 1, -1, 0, 0).finished());
    calib->C_p_I << -0.02, -0.14, 0; 
    calib->image_imu_offset_t = 0;

    calib->sigma_gc = 0;
    calib->sigma_ac = 0;
    calib->sigma_wgc = 0;
    calib->sigma_wac = 0; 

    calib->sigma_Im = 0;

    calib->maxFrame = 5;
    calib->minFrame = 3;

    generator = new DataGenerator(calib);
    msckf = new MSCKF(calib);
    msckf->x.segment<4>(0) = Quaterniond(generator->getRotation()).coeffs();
    msckf->x.segment<3>(0 + 4) = generator->getPosition();
    msckf->x.segment<3>(0 + 4 + 3) = generator->getVelocity();

    cameraMeasurements = new CameraMeasurements();
}

Matrix2Xd cv2eigen(const std::vector<cv::Point2d>& src)
{
    Matrix2Xd dst(2, src.size());
    for (int i = 0; i < src.size(); i++) 
    {
        dst(0, i) = src[i].x;
        dst(1, i) = src[i].y;
    }
    return dst;
}

TEST(CameraTest, testProjectFunctionAndJacobian) 
{
    int n = 10;
    std::vector<cv::Point3d> objectPoints;

    Matrix2Xd ourProjectedPoints(2, n);
    MatrixX3d ourJacobianPoints(2 * n, 3);

    for (int i = 0; i < n; i++)
    {
        Vector3d p = generator->generatePoint();
        p(2) = abs(p(2));
        objectPoints.push_back(cv::Point3d(p(0), p(1), p(2)));
        ourProjectedPoints.col(i) = calib->camera.cameraProject(p);
        ourJacobianPoints.block<2, 3>(i * 2, 0) = calib->camera.jacobianH(p);
    }

    cv::Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix
    K.at<double>(0, 0) = calib->camera.fx;
    K.at<double>(1, 0) = 0;
    K.at<double>(2, 0) = 0;

    K.at<double>(0, 1) = 0;
    K.at<double>(1, 1) = calib->camera.fy;
    K.at<double>(2, 1) = 0;

    K.at<double>(0, 2) = calib->camera.ox;
    K.at<double>(1, 2) = calib->camera.oy;
    K.at<double>(2, 2) = 1;

    cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);
    distCoeffs.at<double>(0) = calib->camera.k1;
    distCoeffs.at<double>(1) = calib->camera.k2;
    distCoeffs.at<double>(2) = calib->camera.t1;
    distCoeffs.at<double>(3) = calib->camera.t2;
    distCoeffs.at<double>(4) = calib->camera.k3;

    cv::Mat rVec(3, 3, cv::DataType<double>::type); // Rotation vector
    rVec.at<double>(0, 0) = 1;
    rVec.at<double>(1, 0) = 0;
    rVec.at<double>(2, 0) = 0;

    rVec.at<double>(0, 1) = 0;
    rVec.at<double>(1, 1) = 1;
    rVec.at<double>(2, 1) = 0;

    rVec.at<double>(0, 2) = 0;
    rVec.at<double>(1, 2) = 0;
    rVec.at<double>(2, 2) = 1;

    cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
    tVec.at<double>(0) = 0;
    tVec.at<double>(1) = 0;
    tVec.at<double>(2) = 0;


    cv::Mat rVecR(3,1,cv::DataType<double>::type);//rodrigues rotation matrix

    cv::Rodrigues(rVec,rVecR);

    std::vector<cv::Point2d> projectedPoints;

    cv::projectPoints(objectPoints, rVecR, tVec, K, distCoeffs, projectedPoints);

    ASSERT_LT((cv2eigen(projectedPoints) - ourProjectedPoints).norm(), 0.1);

    std::vector<cv::Point2d> projectedPoints_x;
    std::vector<cv::Point3d> objectPoints_x;
    std::vector<cv::Point2d> projectedPoints_y;
    std::vector<cv::Point3d> objectPoints_y;
    std::vector<cv::Point2d> projectedPoints_z;
    std::vector<cv::Point3d> objectPoints_z;

    double delta = 0.00000001;
    for (int i = 0; i < n; i++)
    {
        cv::Point3d p = objectPoints[i];
        objectPoints_x.push_back(cv::Point3d(p.x + delta, p.y, p.z));
        objectPoints_y.push_back(cv::Point3d(p.x, p.y + delta, p.z));
        objectPoints_z.push_back(cv::Point3d(p.x, p.y, p.z + delta));
    }
    cv::projectPoints(objectPoints_x, rVecR, tVec, K, distCoeffs, projectedPoints_x);
    cv::projectPoints(objectPoints_y, rVecR, tVec, K, distCoeffs, projectedPoints_y);
    cv::projectPoints(objectPoints_z, rVecR, tVec, K, distCoeffs, projectedPoints_z);

    MatrixXd jacobianPoints_x = (cv2eigen(projectedPoints_x) - cv2eigen(projectedPoints)) / delta;
    MatrixXd jacobianPoints_y = (cv2eigen(projectedPoints_y) - cv2eigen(projectedPoints)) / delta;
    MatrixXd jacobianPoints_z = (cv2eigen(projectedPoints_z) - cv2eigen(projectedPoints)) / delta;

    Map<VectorXd> jacobianPointsCol_x(jacobianPoints_x.data(),jacobianPoints_x.size());
    Map<VectorXd> jacobianPointsCol_y(jacobianPoints_y.data(),jacobianPoints_y.size());
    Map<VectorXd> jacobianPointsCol_z(jacobianPoints_z.data(),jacobianPoints_z.size());

    MatrixX3d jacobianPoints(2 * n, 3);

    jacobianPoints << jacobianPointsCol_x, jacobianPointsCol_y, jacobianPointsCol_z;

    ASSERT_LT((jacobianPoints - ourJacobianPoints).norm(), 1);
}

TEST(CameraTest, testTriangluateFromTrajectory) 
{
    int N = 4;
    Matrix4Xd CG_q(4, N);
    Matrix3Xd G_p_C(3, N);
    Matrix3Xd C_p_f(3, N);
    Matrix2Xd z(2, N);

    for (int i = 0; i < N; i++) 
    {
        generator->setTime((double) i / 4);

        Quaterniond IiG_q = Quaterniond(generator->getRotation());
        Quaterniond CiG_q = IiG_q * calib->CI_q;
        // Calculate camera state
        Vector3d G_p_Ii = generator->getPosition();
        Vector3d G_p_Ci = G_p_Ii - CiG_q * calib->C_p_I;

        G_p_C.col(i) = G_p_Ci;
        CG_q.col(i) = CiG_q.coeffs();
    }
    generator->setTime(0);

    bool valid;
    do
    {
        valid = true;
        Vector3d G_p_f = generator->generatePoint();
        G_p_f = Vector3d(19.000000, 29.000000, 20.000000);
        for (int i = 0; i < N && valid; i++) 
        {
            Quaterniond CiG_q = Quaterniond(CG_q.col(i));
            Vector3d G_p_Ci = G_p_C.col(i);
            Vector3d Ci_p_f = CiG_q.inverse() * (G_p_f - G_p_Ci);
            Vector2d Ci_z_f = calib->camera.cameraProject(Ci_p_f);


            if (abs(atan2(Ci_p_f(0), Ci_p_f(2))) <= M_PI * DataGenerator::FOV / 2 / 180
                    && abs(atan2(Ci_p_f(1), Ci_p_f(2))) <= M_PI * DataGenerator::FOV / 2 / 180
                    && Ci_p_f(2) > 0.1
                    && 0 < Ci_z_f(0) && Ci_z_f(0) < DataGenerator::COL 
                    && 0 < Ci_z_f(1) && Ci_z_f(1) < DataGenerator::ROW)
            {
                CG_q.col(i) = CiG_q.coeffs();
                G_p_C.col(i) = G_p_Ci;
                C_p_f.col(i) = Ci_p_f;
                z.col(i) = Ci_z_f;
            }
            else 
            {
                cout << "wrong G_p_f" << Ci_p_f.transpose() << "z_f" << Ci_z_f.transpose() << endl;
                cout << abs(atan2(Ci_p_f(0), Ci_p_f(2))) * 180 / M_PI * 2 << endl;
                cout << abs(atan2(Ci_p_f(1), Ci_p_f(2))) * 180 / M_PI * 2 << endl;
                
                valid = false;
            }
        }
        if (valid) 
        {
            cout << "G_p_f:" << G_p_f.transpose() << endl;
            Vector3d G_p_f_hat = calib->camera.triangulate(CG_q, G_p_C, z);

            cout << ">>> G_p_f_hat:" << G_p_f_hat.transpose() << endl;        
        }
    } 
    while (!valid);
}

TEST(CameraTest,  DISABLED_testTriangluate) 
{
    generator->setTime(0);
    int testNum = 100;
    VectorXd error(testNum);
    while (testNum--)
    {
        Vector3d G_p_f = generator->generatePoint();
        int N = 10;
        Matrix4Xd CG_q(4, N);
        Matrix3Xd G_p_C(3, N);
        Matrix3Xd C_p_f(3, N);
        Matrix2Xd z(2, N);

        int n = 0;
        while (n < N)
        {
            Quaterniond CiG_q = Quaterniond(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5);
            CiG_q.normalize();
            Vector3d G_p_Ci = generator->generatePoint();
            Vector3d Ci_p_f = CiG_q.inverse() * (G_p_f - G_p_Ci);
            Vector2d Ci_z_f = calib->camera.cameraProject(Ci_p_f);

            if (abs(atan2(Ci_p_f(0), Ci_p_f(2))) <= M_PI * DataGenerator::FOV / 2 / 180
                    && abs(atan2(Ci_p_f(1), Ci_p_f(2))) <= M_PI * DataGenerator::FOV / 2 / 180
                    && Ci_p_f(2) > 0.1
                    && 0 < Ci_z_f(0) && Ci_z_f(0) < DataGenerator::COL 
                    && 0 < Ci_z_f(1) && Ci_z_f(1) < DataGenerator::ROW)
            {
                CG_q.col(n) = CiG_q.coeffs();
                G_p_C.col(n) = G_p_Ci;
                C_p_f.col(n) = Ci_p_f;
                z.col(n) = Ci_z_f;
                n++;
            }
        }
        // cout << "=======" << endl;
        // cout << "CG_q:" << endl << CG_q << endl;
        // cout << "G_p_C:" << endl << G_p_C << endl;
        // cout << "C_p_f:" << endl << C_p_f << endl;
        // cout << "G_z_f:" << endl << z << endl;
        //cout << "G_p_f:" << G_p_f.transpose() << endl;

        Vector3d G_p_f_hat = calib->camera.triangulate(CG_q, G_p_C, z);
        //cout << ">>> G_p_f_hat:" << G_p_f_hat.transpose() << endl;

        error[testNum] = (G_p_f - G_p_f_hat).norm();
    }
    //cout << "error: " << error.mean() << endl;
    ASSERT_LT(error.mean(), 0.5);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    setup();

    return RUN_ALL_TESTS();
}

