#include <cstdlib>
#include <cmath>
#include <vector>
#include <tuple>
#include <map>
#include <algorithm>
#include <random>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;


class DataGenerator
{
public:
    DataGenerator();
    void update();

    double getTime();

    Vector3d getPoint(int i);
    vector<Vector3d> getCloud();
    Vector3d getPosition();
    Matrix3d getRotation();
    Vector3d getVelocity();

    Vector3d getAngularVelocity();
    Vector3d getLinearAcceleration();

    vector<pair<int, Vector3d>> getImage();

    static int const NUM_POINTS  = 100;
    static int const MAX_BOX     = 10;
    static int const FREQ        = 100;
    static int const IMU_PER_IMG = 50;
    static int const MAX_TIME    = 10;
    static int const FOV         = 90;
    static double constexpr PI   = M_PI;//3.1415926;//acos(-1);
private:
    int pts[NUM_POINTS * 3];
    double t;
    std::map<int, int> before_feature_id;
    std::map<int, int> current_feature_id;
    int current_id;

    Matrix3d Ric;
    Vector3d Tic;
    Matrix3d acc_cov, gyr_cov;
    Matrix2d pts_cov;
    default_random_engine generator;
    normal_distribution<double> distribution;
};
