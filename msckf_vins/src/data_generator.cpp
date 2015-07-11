#include "data_generator.h"
#define PI 3.1415926
//#define WITH_NOISE 1 
#define COMPLEX_TRAJECTORY 1
DataGenerator::DataGenerator()
{
    srand(0);
    t = 0;
    current_id = 0;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        pts[i](0) = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i](1) = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i](2) = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
    }
    R_cb << 0, -1, 0,
            0, 0, 1,
            -1, 0, 0;
    //Tic << 4, 5, 6;
    p_cb << -0.14, -0.02, 0;

    //acc_cov << 1.3967e-04, 1.4357e-06, 2.1468e-06,
    //        1.4357e-06, 1.4352e-04, 5.7168e-05,
    //        2.1468e-06, 5.7168e-05, 1.5757e-04;
    //acc_cov << 1.3967e-04, 0, 0,
    //        0, 1.4352e-04, 0,
    //        0, 0, 1.5757e-04;
    acc_cov = Matrix3d::Identity();
    gyr_cov =  Matrix3d::Identity();


    pts_cov << .1 * .1 / 3.6349576068362910e+02 / 3.6349576068362910e+02, 0,
            0, .1 * .1 / 3.6356654972681025e+02 / 3.6356654972681025e+02;

    generator = default_random_engine(0);
    distribution = normal_distribution<double>(0.0, 1);
}



void DataGenerator::update()
{
    t += 1.0 / FREQ;
}

double DataGenerator::getTime()
{
    return t;
}

Vector3d DataGenerator::getPoint(int i)
{
    return pts[i];
}

Vector3d DataGenerator::getPosition()
{
#ifdef COMPLEX_TRAJECTORY
    double x, y, z;
    if (t < MAX_TIME)
    {
        x = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * 2 * 2);
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * 2);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        x = MAX_BOX / 2.0 - MAX_BOX / 2.0;
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0;
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        x = -MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * 2 * 2);
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * 2);
    }
#elif SIMPLE_TRAJECTORY
    double x, y, z;
    x = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI);
    y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * 2 * 2);
    z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * 2);
#else
    double x = 10 * cos(t / 10);
    double y = 10 * sin(t / 10);
    double z = 3;
#endif

    return Vector3d(x, y, z);
}

Matrix3d DataGenerator::getRotation()
{
#ifdef COMPLEX_TRAJECTORY
    return (AngleAxisd(30.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitX())
            * AngleAxisd(40.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitY())
            * AngleAxisd(0, Vector3d::UnitZ())).toRotationMatrix();
#elif SIMPLE_TRAJECTORY
    return (AngleAxisd(0.0 + M_PI * sin(t / 10), Vector3d::UnitY()) * AngleAxisd(0.0 + M_PI * sin(t / 10), Vector3d::UnitX())).toRotationMatrix();
#else
    return Matrix3d::Identity();
#endif
}

Vector3d DataGenerator::getAngularVelocity()
{
    const double delta_t = 0.00001;
    Matrix3d rot = getRotation();
    t += delta_t;
    Matrix3d drot = (getRotation() - rot) / delta_t;
    t -= delta_t;
    Matrix3d skew = rot.inverse() * drot;
#ifdef WITH_NOISE
    Vector3d disturb = Vector3d(distribution(generator) * sqrt(gyr_cov(0, 0)),
                                distribution(generator) * sqrt(gyr_cov(1, 1)),
                                distribution(generator) * sqrt(gyr_cov(2, 2))
                               );
    return disturb + Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#else
    return Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#endif
}

Vector3d DataGenerator::getVelocity()
{
#ifdef COMPLEX_TRAJECTORY
    double dx, dy, dz;
    if (t < MAX_TIME)
    {
        dx = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        dz = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        dx = 0.0;
        dy = 0.0;
        dz = 0.0;
    }
    else
    {
        double tt = t -  2 * MAX_TIME;
        dx = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        dz = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);
    }
#elif SIMPLE_TRAJECTORY
    double dx, dy, dz;
    dx = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
    dy = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
    dz = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);
#else
    double dx = - sin(t / 10);
    double dy = cos(t / 10);
    double dz = 0;
#endif
    return Vector3d(dx, dy, dz);
}

//I_am = GI_R(G_a - G_g), G_g = (0, 0, -9.8)
Vector3d DataGenerator::getLinearAcceleration()
{
    double ddx, ddy, ddz;
    if (t < MAX_TIME)
    {
        ddx = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        ddz = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        ddx = 0.0;
        ddy = 0.0;
        ddz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        ddx = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        ddz = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);
    }
#if WITH_NOISE
    Vector3d disturb = Vector3d(distribution(generator) * sqrt(acc_cov(0, 0)),
                                distribution(generator) * sqrt(acc_cov(1, 1)),
                                distribution(generator) * sqrt(acc_cov(2, 2))
                               );
    return getRotation().inverse() * (disturb + Vector3d(ddx, ddy, ddz + 9.8));
#else
    return getRotation().inverse() * Vector3d(ddx, ddy, ddz + 9.8);
#endif
}

vector<pair<int, Vector3d>> DataGenerator::getImage()
{
    vector<pair<int, Vector3d>> image;
    Vector3d position = getPosition();
    Matrix3d R_gb = getRotation();           //R_gb
    printf("===frame start===\n");

    printf("max: %d\n", current_id);

    for (int i = 0; i < NUM_POINTS; i++)
    {
        Vector3d local_point = R_cb * R_gb.transpose() * (pts[i] - position) + p_cb;

        double xx = local_point(0);
        double yy = local_point(1);
        double zz = local_point(2);

        if (abs(atan2(xx, zz)) <= M_PI * FOV / 2 / 180
                && abs(atan2(yy, zz)) <= M_PI * FOV / 2 / 180
                && zz > 0)
        {
            //int n_id = before_feature_id.find(i) == before_feature_id.end() ?
            //           current_id++ : before_feature_id[i];
            int n_id = i;
#if WITH_NOISE
           Vector3d disturb = Vector3d(distribution(generator) * sqrt(pts_cov(0, 0)) / zz / zz,
                                       distribution(generator) * sqrt(pts_cov(1, 1)) / zz / zz,
                                       0
                                      );
           image.push_back(make_pair(n_id, disturb + Vector3d(xx / zz, yy / zz, 1)));
#else
            image.push_back(make_pair(n_id, local_point));
            // printf("feature id: %d, p_gf: [%f, %f, %f], p_cf: [%f, %f, %f]\n",
            //        n_id, pts[i](0), pts[i](1), pts[i](2), local_point(0), local_point(1), local_point(2));

#endif
            current_feature_id[i] = n_id;
        }
    }
    printf("===frame end===\n");
    before_feature_id = current_feature_id;
    current_feature_id.clear();
    sort(image.begin(), image.end(), [](const pair<int, Vector3d> &a,
                                        const pair<int, Vector3d> &b)
        {
            return a.first < b.first;
        });
    return image;
}

vector<Vector3d> DataGenerator::getCloud()
{
    vector<Vector3d> cloud;

    for (int i = 0; i < NUM_POINTS; i++)
    {
        cloud.push_back(pts[i]);
    }
    return cloud;
}
