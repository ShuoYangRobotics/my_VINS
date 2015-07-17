#include "camera.h"
#include <limits>
#include <unsupported/Eigen/NonLinearOptimization>

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

Vector3d Camera::triangulateGN(const Matrix4Xd& CG_q, const Matrix3Xd& G_p_C, const Matrix2Xd& z, const Vector3d& guess)
{
    int iterations = 1000;

    assert (CG_q.cols() == G_p_C.cols());
    assert (CG_q.cols() == z.cols());

    unsigned int n = z.cols();

    Vector3d G_p_f = guess;

    // Residual
    VectorXd r(n * 2);
    // Jacobian
    MatrixX3d J(n * 2, 3);

    while (iterations--) 
    {
        for (int i = 0; i < n; i++) 
        {
            Matrix3d GCi_R = Quaterniond(CG_q.col(i)).inverse().toRotationMatrix();
            Vector3d Ci_p_f = GCi_R * (G_p_f - G_p_C.col(i));
            Matrix3d Jg = GCi_R;

            r.segment<2>(i * 2) = z.col(i) - cameraProject(Ci_p_f);
            J.block<2, 3>(i * 2, 0) = -jacobianH(Ci_p_f) * Jg;
        }

        if (r.norm() < 0.001) 
            break;
        // New estimate
        G_p_f = G_p_f - (J.transpose() * J).inverse() * J.transpose() * r;
    }

    return G_p_f;
}

Vector3d Camera::triangulateGNInverseDepth(const Matrix4Xd& CG_q, const Matrix3Xd& G_p_C, const Matrix2Xd& z, const Vector3d& guess)
{
    int iterations = 1000;

    assert (CG_q.cols() == G_p_C.cols());
    assert (CG_q.cols() == z.cols());

    unsigned int n = z.cols();

    // Residual
    VectorXd r(n * 2);
    // Jacobian
    MatrixX3d J(n * 2, 3);

    Vector3d C0_guess = Quaterniond(CG_q.col(0)).inverse() * (guess - G_p_C.col(0));
    Vector3d theta(C0_guess(0) / C0_guess(2), C0_guess(1) / C0_guess(2), 1.0 / C0_guess(2));
    while (iterations--) 
    {

        for (int i = 0; i < n; i++) 
        {
            // Calculate i-th camera transition to camera 0
            Quaterniond GCi_q = Quaterniond(CG_q.col(i)).inverse();
            Quaterniond C0G_q = Quaterniond(CG_q.col(0));

            Matrix3d C0Ci_R = (GCi_q * C0G_q).toRotationMatrix();
            Vector3d Ci_p_C0 = GCi_q * (G_p_C.col(0) - G_p_C.col(i));

            // Calculate inverse depth
            Vector3d Ci_p_f = C0Ci_R * Vector3d(theta(0), theta(1), 1) + theta(2) * Ci_p_C0;
            Matrix3d Jg; 
            Jg << C0Ci_R.col(0), C0Ci_R.col(1), Ci_p_C0;

            r.segment<2>(i * 2) = z.col(i) - cameraProject(Ci_p_f);
            J.block<2, 3>(i * 2, 0) = -jacobianH(Ci_p_f) * Jg;
        }

        if (r.norm() < 0.001) 
            break;
        // New estimate
        theta = theta - (J.transpose() * J).inverse() * J.transpose() * r;
        
    }

    Vector3d C0_p_f(theta(0) / theta(2), theta(1) / theta(2), 1.0 / theta(2));

    Vector3d G_p_f = Quaterniond(CG_q.col(0)) * C0_p_f + G_p_C.col(0);

    return G_p_f;
}


struct TriangulateFunctor
{
    int n;

    Camera* camera;
    Matrix4Xd CG_q;
    Matrix3Xd G_p_C;
    Matrix2Xd z;

    TriangulateFunctor(Camera* _camera, const Matrix4Xd& _CG_q, const Matrix3Xd& _G_p_C, const Matrix2Xd& _z)
        : camera(_camera), CG_q(_CG_q), G_p_C(_G_p_C), z(_z)
    {
        n = z.cols();
    }

    int operator()(const VectorXd &G_p_f, VectorXd &r) const
    {
        for (int i = 0; i < n; i++) 
        {
            Matrix3d GCi_R = Quaterniond(CG_q.col(i)).inverse().toRotationMatrix();
            Vector3d Ci_p_f = GCi_R * (G_p_f - G_p_C.col(i));

            r.segment<2>(i * 2) = z.col(i) - camera->cameraProject(Ci_p_f);
        }

        return 0;
    }

    int df(const VectorXd &G_p_f, MatrixXd &J) const
    {
        for (int i = 0; i < n; i++) 
        {
            Matrix3d GCi_R = Quaterniond(CG_q.col(i)).inverse().toRotationMatrix();
            Vector3d Ci_p_f = GCi_R * (G_p_f - G_p_C.col(i));
            Matrix3d Jg = GCi_R;

            J.block<2, 3>(i * 2, 0) = -camera->jacobianH(Ci_p_f) * Jg;
        }
        return 0;
    }

    int inputs() const { return 3; }
    int values() const { return n * 2; } 
};

Vector3d Camera::triangulateLM(const Matrix4Xd& CG_q, const Matrix3Xd& G_p_C, const Matrix2Xd& z, const Vector3d& guess)
{
    TriangulateFunctor functor(this, CG_q, G_p_C, z);
    LevenbergMarquardt<TriangulateFunctor, double> lm(functor);
    VectorXd G_p_f = guess;
    LevenbergMarquardtSpace::Status status = lm.minimize(G_p_f);
    return G_p_f;
}

struct InverseDepthTriangulateFunctor
{
    int n;

    Camera* camera;
    Matrix4Xd CG_q;
    Matrix3Xd G_p_C;
    Matrix2Xd z;

    InverseDepthTriangulateFunctor(Camera* _camera, const Matrix4Xd& _CG_q, const Matrix3Xd& _G_p_C, const Matrix2Xd& _z)
        : camera(_camera), CG_q(_CG_q), G_p_C(_G_p_C), z(_z)
    {
        n = z.cols();
    }

    int operator()(const VectorXd &theta, VectorXd &r) const
    {
        for (int i = 0; i < n; i++) 
        {
            Quaterniond GCi_q = Quaterniond(CG_q.col(i)).inverse();
            Quaterniond C0G_q = Quaterniond(CG_q.col(0));

            Matrix3d C0Ci_R = (GCi_q * C0G_q).toRotationMatrix();
            Vector3d Ci_p_C0 = GCi_q * (G_p_C.col(0) - G_p_C.col(i));

            // Calculate inverse depth
            Vector3d Ci_p_f = C0Ci_R * Vector3d(theta(0), theta(1), 1) + theta(2) * Ci_p_C0;

            r.segment<2>(i * 2) = z.col(i) - camera->cameraProject(Ci_p_f);
        }

        return 0;
    }

    int df(const VectorXd &theta, MatrixXd &J) const
    {
        for (int i = 0; i < n; i++) 
        {
             // Calculate i-th camera transition to camera 0
            Quaterniond GCi_q = Quaterniond(CG_q.col(i)).inverse();
            Quaterniond C0G_q = Quaterniond(CG_q.col(0));

            Matrix3d C0Ci_R = (GCi_q * C0G_q).toRotationMatrix();
            Vector3d Ci_p_C0 = GCi_q * (G_p_C.col(0) - G_p_C.col(i));

            // Calculate inverse depth
            Vector3d Ci_p_f = C0Ci_R * Vector3d(theta(0), theta(1), 1) + theta(2) * Ci_p_C0;
            Matrix3d Jg; 
            Jg << C0Ci_R.col(0), C0Ci_R.col(1), Ci_p_C0;

            J.block<2, 3>(i * 2, 0) = -camera->jacobianH(Ci_p_f) * Jg;
        }
        return 0;
    }

    int inputs() const { return 3; }
    int values() const { return n * 2; } 
};

Vector3d Camera::triangulateLMInverseDepth(const Matrix4Xd& CG_q, const Matrix3Xd& G_p_C, const Matrix2Xd& z, const Vector3d& guess)
{
    InverseDepthTriangulateFunctor functor(this, CG_q, G_p_C, z);
    LevenbergMarquardt<InverseDepthTriangulateFunctor, double> lm(functor);
    Vector3d C0_guess = Quaterniond(CG_q.col(0)).inverse() * (guess - G_p_C.col(0));
    VectorXd theta = Vector3d(C0_guess(0) / C0_guess(2), C0_guess(1) / C0_guess(2), 1.0 / C0_guess(2));
    LevenbergMarquardtSpace::Status status = lm.minimize(theta);
    Vector3d C0_p_f(theta(0) / theta(2), theta(1) / theta(2), 1.0 / theta(2));
    Vector3d G_p_f = Quaterniond(CG_q.col(0)) * C0_p_f + G_p_C.col(0);
    return G_p_f;
}

Vector3d Camera::triangulateFromTwoView(const Quaterniond& CiG_q, const Vector3d& G_p_Ci, const Vector2d& zi,
                                        const Quaterniond& CjG_q, const Vector3d& G_p_Cj, const Vector2d& zj)
{
    Vector3d Ci_p_f((zi(0) - ox) / fx, (zi(1) - oy) / fy, 1);
    Vector3d Cj_p_f((zj(0) - ox) / fx, (zj(1) - oy) / fy, 1); 

    Vector3d Ci_d_Cjf = CiG_q.inverse() * CjG_q * Cj_p_f;
    Vector3d Ci_p_Cj = CiG_q.inverse() * (G_p_Cj - G_p_Ci);
    
    //Solve depth_i * Ci_p_f == depth_j * Ci_d_Cjf + Ci_p_Cj
    //  depth_i = Ci_d_Cjf.cross(Ci_p_Cj) / Ci_d_Cjf.cross(Ci_p_f)
    //  depth_j = Ci_p_f.cross(Ci_p_Cj) / Ci_d_Cjf.cross(Ci_p_f)

    double depth_i = Ci_d_Cjf.cross(Ci_p_Cj)(2) / Ci_d_Cjf.cross(Ci_p_f)(2);
    Ci_p_f *= depth_i;
    // double depth_j = Ci_p_f.cross(Ci_p_Cj)(2) / Ci_d_Cjf.cross(Ci_p_f)(2);
    // Cj_p_f *= depth_j;

    Vector3d G_p_f = CiG_q * Ci_p_f + G_p_Ci;

    return G_p_f;
}

Vector3d Camera::triangulate(const Matrix4Xd& CG_q, const Matrix3Xd& G_p_C, const Matrix2Xd& z, double* r_norm)
{
    int n = z.cols();
    TriangulateFunctor functor(this, CG_q, G_p_C, z);
    Vector3d G_p_f;
    double min_r_norm = numeric_limits<double>::infinity(); 

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            if (i != j) 
            {
                Vector3d G_p_f_guess = triangulateFromTwoView(
                        Quaterniond(CG_q.col(i)), G_p_C.col(i), z.col(i), 
                        Quaterniond(CG_q.col(j)), G_p_C.col(j), z.col(j));
               
                Vector3d G_p_f_hat = triangulateLM(CG_q, G_p_C, z, G_p_f_guess);
                VectorXd r(n * 2);
                functor(G_p_f_hat, r);
                if (r.norm() < min_r_norm) 
                {
                    min_r_norm = r.norm();
                    G_p_f = G_p_f_hat;
                }
            }

    if (r_norm != NULL)
        *r_norm = min_r_norm;
    
    // cout << "===triangulate====" << endl;
    // cout << "CG_q:" << endl << CG_q << endl;
    // cout << "G_p_C:" << endl << G_p_C << endl;
    // cout << "G_z_f:" << endl << z << endl;
    // cout << "G_p_f:" << G_p_f.transpose() << endl;

    return G_p_f;
}

ostream& operator<<(ostream& out, const Camera& camera)
{
    return out <<
           "ox: " << camera.ox << " oy: " << camera.oy << "\n" <<
           "fx: " << camera.fx << " fy: " << camera.fy << "\n" <<
           "k1: " << camera.k1 << " k2: " << camera.k2 << "\n" <<
           "t1: " << camera.t1 << " t2: " << camera.t2;
}
