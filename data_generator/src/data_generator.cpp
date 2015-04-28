#include "data_generator.h"

//#define WITH_NOISE 

DataGenerator::DataGenerator()
{
    srand(0);
    t = 0;
    current_id = 0;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        pts[i * 3 + 0] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 1] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 2] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
    }
    Ric <<
        0, 0, -1,
        -1, 0, 0,
        0, 1, 0;
    //Tic << 4, 5, 6;
    Tic << 0.02, -0.14, 0.0;
    //acc_cov << 1.3967e-04, 1.4357e-06, 2.1468e-06,
    //        1.4357e-06, 1.4352e-04, 5.7168e-05,
    //        2.1468e-06, 5.7168e-05, 1.5757e-04;
    //acc_cov << 1.3967e-04, 0, 0,
    //        0, 1.4352e-04, 0,
    //        0, 0, 1.5757e-04;
    acc_cov = 1e-2 * Matrix3d::Identity();
    gyr_cov = 1e-4 * Matrix3d::Identity();


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
    return Vector3d(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]);
}

Vector3d DataGenerator::getPosition()
{
    //double backup = t;
    //if (t < MAX_TIME)
    //    t = 0;
    //else if (t > 2 * MAX_TIME)
    //    t = 2 * MAX_TIME;

    //double x = pow(t, 3) / pow(MAX_TIME, 3) * MAX_BOX;
    //double y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * PI * 2 * 2);
    //double z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * PI * 2);
    double x = 10*cos(t/10);
    double y = 10*sin(t/10);
    double z = 3;

    return Vector3d(x, y, z);
}

Matrix3d DataGenerator::getRotation()
{
    //return (AngleAxisd(30.0 / 180 * PI * sin(t / MAX_TIME * PI * 2), Vector3d::UnitX())
    //        * AngleAxisd(40.0 / 180 * PI * sin(t / MAX_TIME * PI * 2), Vector3d::UnitY())
    //        * AngleAxisd(0, Vector3d::UnitZ())).toRotationMatrix();
    //return AngleAxisd(30.0 / 180 * PI * sin(t / MAX_TIME * PI * 2), Vector3d::UnitZ()).toRotationMatrix();
    return Matrix3d::Identity();
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
    //double dx = 3 * pow(t, 2) / pow(MAX_TIME, 3) * MAX_BOX;
    //double dy = MAX_BOX / 2.0 * -sin(t / MAX_TIME * PI * 2 * 2) * (1.0 / MAX_TIME * PI * 2 * 2);
    //double dz = MAX_BOX / 2.0 * -sin(t / MAX_TIME * PI * 2) * (1.0 / MAX_TIME * PI * 2);
    double dx = -sin(t/10);
    double dy =  cos(t/10);
    double dz =  0;

    return getRotation().inverse() * Vector3d(dx, dy, dz);
}

Vector3d DataGenerator::getLinearAcceleration()
{
    //double ddx = 2 * 3 * pow(t, 1) / pow(MAX_TIME, 3) * MAX_BOX;
    //double ddy = MAX_BOX / 2.0 * -cos(t / MAX_TIME * PI * 2 * 2) * (1.0 / MAX_TIME * PI * 2 * 2) * (1.0 / MAX_TIME * PI * 2 * 2);
    //double ddz = MAX_BOX / 2.0 * -cos(t / MAX_TIME * PI * 2) * (1.0 / MAX_TIME * PI * 2) * (1.0 / MAX_TIME * PI * 2);
    double ddx = -0.1*cos(t/10);
    double ddy = -0.1*sin(t/10);
    double ddz = 0;
#ifdef WITH_NOISE
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
    Matrix3d quat = getRotation();           //R_gb
    printf("max: %d\n", current_id);
    for (int i = 0; i < NUM_POINTS; i++)
    {
        double xx = pts[i * 3 + 0] - position(0);
        double yy = pts[i * 3 + 1] - position(1);
        double zz = pts[i * 3 + 2] - position(2);
        Vector3d local_point = Ric.inverse() * (quat.inverse() * Vector3d(xx, yy, zz) - Tic);
        xx = local_point(0);
        yy = local_point(1);
        zz = local_point(2);

        if (std::fabs(atan2(xx, zz)) <= PI * FOV / 2 / 180
                && std::fabs(atan2(yy, zz)) <= PI * FOV / 2 / 180
                && zz > 0)
        {
            int n_id = before_feature_id.find(i) == before_feature_id.end() ?
                       current_id++ : before_feature_id[i];
//#if WITH_NOISE
//            Vector3d disturb = Vector3d(distribution(generator) * sqrt(pts_cov(0, 0)) / zz / zz,
//                                        distribution(generator) * sqrt(pts_cov(1, 1)) / zz / zz,
//                                        0
//                                       );
//            image.push_back(make_pair(n_id, disturb + Vector3d(xx / zz, yy / zz, 1)));
//#else
            image.push_back(make_pair(n_id, Vector3d(xx, yy, zz)));
            printf ("id %d, (%d %d %d)\n", n_id,
                pts[i * 3 + 0] ,
                pts[i * 3 + 1] ,
                pts[i * 3 + 2] 
            );
                            
//#endif
            current_feature_id[i] = n_id;
        }
    }
    before_feature_id = current_feature_id;
    current_feature_id.clear();
    //sort(image.begin(), image.end(), [](const pair<int, Vector3d> &a,
    //                                    const pair<int, Vector3d> &b)
    //{
    //    return a.first < b.first;
    //});
    return image;
}

vector<Vector3d> DataGenerator::getCloud()
{
    vector<Vector3d> cloud;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        double xx = pts[i * 3 + 0];
        double yy = pts[i * 3 + 1];
        double zz = pts[i * 3 + 2];
        cloud.push_back(Vector3d(xx, yy, zz));
    }
    return cloud;
}
