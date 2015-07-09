#include "camera.h"

Camera::Camera()
{
    /* Projection */
    ox = 0; oy = 0;          // principal point [px,px]
    fx = 1; fy = 1;          // focal length [px,px]
    k1 = 0; k2 = 0; k3 = 0;  // radial distortion parameters [n/u,n/u]
    t1 = 0; t2 = 0;          // tangential distortion parameters [n/u,n/u]
}

void Camera::setIntrinsicParam(double _fx, double _fy, double _ox, double _oy)
{
    fx = _fx;
    fy = _fy;
    ox = _ox;
    oy = _oy;
}

void Camera::setDistortionParam(double _k1, double _k2, double _t1, double _t2, double _k3)
{
    k1 = _k1;
    k2 = _k2;
    k3 = _k3;
    t1 = _t1;
    t2 = _t2;
}

//
// Project a point from camera coordinates to pixel position
//
Vector2d Camera::cameraProject(const Vector3d& p)
{
    double x = p(0);
    double y = p(1);
    double z = p(2);
    double u = x / z;
    double v = y / z;
    double r2 = u * u + v * v;
    double dr = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    Vector2d dt(
        2 * u * v * t1 + (r2 + 2 * u * u) * t2,
        2 * u * v * t2 + (r2 + 2 * v * v) * t1
    );

    return Vector2d(ox + fx * (dr * u + dt(0)), oy + fy * (dr * v + dt(1)));
}

//
// Jacobian of h (camera model)
//
Matrix<double, 2, 3> Camera::jacobianH(const Vector3d& p)
{
    double x = p(0);
    double y = p(1);
    double z = p(2);

    double u = x / z;
    double v = y / z;

    double r2 = u * u + v * v;

    double dr = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    Vector2d dt(
        2 * u * v * t1 + (r2 + 2 * u * u) * t2,
        2 * u * v * t2 + (r2 + 2 * v * v) * t1
    );
    
    Vector3d Jdr = 2 / z * (k1 + 2 * k2 * r2 + 3 * k3 * r2 * r2) * Vector3d(u, v, -r2);

    Matrix<double, 2, 3> Jdt;
    Jdt << 3 * t2 * u + t1 * v, t1 * u + t2 * v, -2 * t1 * u * v - t2 * (3 * u * u + v * v),
          t1 * u + t2 * v, t2 * u + 3 * t1 * v, -2 * t2 * u * v - t1 * (u * u + 3 * v * v);
    Jdt *= 2 / z;

    Matrix<double, 2, 3> Jdistorted;
    Jdistorted = dr / z * (Matrix<double, 2, 3>() << 1, 0, -u, 0, 1, -v).finished() + Vector2d(u, v) * Jdr.transpose() + Jdt;
    
    Matrix<double, 2, 3> J = Vector2d(fx, fy).asDiagonal() * Jdistorted;

    return J;
}

ostream& operator<<(ostream& out, const Camera& camera)
{
    return out <<
           "ox: " << camera.ox << " oy: " << camera.oy << "\n" <<
           "fx: " << camera.fx << " fy: " << camera.fy << "\n" <<
           "k1: " << camera.k1 << " k2: " << camera.k2 << "\n" <<
           "t1: " << camera.t1 << " t2: " << camera.t2;
}
