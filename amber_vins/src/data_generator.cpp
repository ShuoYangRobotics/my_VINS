#include "data_generator.h"
#define COMPLEX_TRAJECTORY 1
//#define SIMPLE_TRAJECTORY 1

//#define WITH_NOISE 

Vector3d DataGenerator::generatePoint()
{
    return Vector3d(rand() % (6 * MAX_BOX) - 3 * MAX_BOX,
                    rand() % (6 * MAX_BOX) - 3 * MAX_BOX,
                    rand() % (6 * MAX_BOX) - 3 * MAX_BOX);
}

DataGenerator::DataGenerator(Calib* _calib): calib(_calib)
{
    srand(0);
    t = 0;
    current_id = 0;
    for (int i = 0; i < NUM_POINTS; i++)
        pts[i] = generatePoint();
    random_generator = default_random_engine(0);
    distribution = normal_distribution<double>(0, 1);
}

void DataGenerator::update()
{
    t += 1.0 / FREQ;
}

double DataGenerator::getTime()
{
    return t;
}

void DataGenerator::setTime(double _t)
{
    t = _t;
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
//    return (AngleAxisd(0.0 + M_PI * sin(t / 10), Vector3d::UnitY()) * AngleAxisd(0.0 + M_PI * sin(t / 10), Vector3d::UnitX())).toRotationMatrix();
//    return Matrix3d::Identity();

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

Vector3d DataGenerator::getIMUAngularVelocity()
{
    const double delta_t = 0.00001;
    Matrix3d rot = getRotation();
    t += delta_t;
    Matrix3d drot = (getRotation() - rot) / delta_t;
    t -= delta_t;
    Matrix3d skew = rot.inverse() * drot;
#ifdef WITH_NOISE
    Vector3d disturb = Vector3d(distribution(random_generator) * calib->sigma_gc,
                                distribution(random_generator) * calib->sigma_gc,
                                distribution(random_generator) * calib->sigma_gc
                               );
    return disturb + Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#else
    return Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#endif
}

Vector3d DataGenerator::getIMULinearAcceleration()
{
#ifdef COMPLEX_TRAJECTORY
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
#elif SIMPLE_TRAJECTORY
    double ddx, ddy, ddz;
    ddx = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
    ddy = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
    ddz = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);
#else
    double ddx = -0.1 * cos(t / 10);
    double ddy = -0.1 * sin(t / 10);
    double ddz = 0;
#endif

#if WITH_NOISE
    Vector3d disturb = Vector3d(distribution(random_generator) * calib->sigma_ac,
                                distribution(random_generator) * calib->sigma_ac,
                                distribution(random_generator) * calib->sigma_ac);
#else
    Vector3d disturb = Vector3d(0, 0, 0);
#endif
    return getRotation().transpose() * (disturb + Vector3d(ddx, ddy, ddz + calib->g));
}

vector<pair<int, Vector2d>> DataGenerator::getImage()
{
    printf("===frame start===\n");

    vector<pair<int, Vector2d>> image;

    Vector3d position = getPosition();
    Matrix3d GI_R = getRotation().transpose(); 

    for (int i = 0; i < NUM_POINTS; i++)
    {
        Vector3d C_p_f = calib->CI_q.toRotationMatrix().transpose() * GI_R * (pts[i] - position) + calib->C_p_I;
        Vector2d C_z_f = calib->camera.cameraProject(C_p_f);

        if (abs(atan2(C_p_f(0), C_p_f(2))) <= M_PI * FOV / 2 / 180
                && abs(atan2(C_p_f(1), C_p_f(2))) <= M_PI * FOV / 2 / 180
                && C_p_f(2) > 0.1
                && 0 < C_z_f(0) && C_z_f(0) < COL && 0 < C_z_f(1) &&  C_z_f(1) < ROW)
        {
           // int n_id = before_feature_id.find(i) == before_feature_id.end() ?
           //            current_id++ : before_feature_id[i];
            int n_id = i;
#if WITH_NOISE
           Vector2d disturb = Vector2d(distribution(random_generator) * calib->sigma_Im / C_p_f(2) / C_p_f(2),
                                       distribution(random_generator) * calib->sigma_Im / C_p_f(2) / C_p_f(2));
#else
           Vector2d disturb = Vector2d(0, 0);
#endif
           image.push_back(make_pair(n_id, disturb + C_z_f));

           printf("Feature Id: %d, p_gf: [%f, %f, %f], p_cf: [%f, %f, %f], z_cf: [%f, %f]\n",
                 n_id, pts[i](0), pts[i](1), pts[i](2), C_p_f(0), C_p_f(1), C_p_f(2), C_z_f(0), C_z_f(1));

           current_feature_id[i] = n_id;
        }
    }
    printf("===frame end===\n");
    /*before_feature_id = current_feature_id;
    current_feature_id.clear();
    */
    sort(image.begin(), image.end(), [](const pair<int, Vector2d> &a,
                                        const pair<int, Vector2d> &b)
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
