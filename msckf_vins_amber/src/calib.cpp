#include "calib.h"
#include "odometry.h"

Calib::Calib()
{
    //
    // Camera calibration
    //
    camera = Camera();
   
    /* Position */
    CI_q = Quaterniond(1, 0, 0, 0); // Rotation from camera to IMU coordinates. [unit quaternion]
    C_p_I = Vector3d(0, 0, 0);      // Position of inertial frame in camera coordinates [m,m,m]
    //
    // Physical properties
    //
    g = 9.8;                        // Gravitational acceleration [m/s^2]
    delta_t = 1;                    // Time between IMU data [s]
    image_imu_offset_t = 0;         //Time offset beween Image and IMU [s]
    //
    // Noise levels
    //
    sigma_gc = 0; sigma_ac = 0;    // Standard deviation of IMU noise [rad/s, m/s^2]
    sigma_wgc = 0; sigma_wac = 0;  // Standard deviation of IMU random walk noise [rad/s, m/s^2]
    /* Camera */
    sigma_Im = 0;                  // Image noise
    //
    // Options
    //
    maxFrame = ODO_MAX_FRAMES;     // Maximum frames in FIFO
    minFrame = 0;
}

ostream& operator<<(ostream& out, const Calib& calib)
{
    return out << calib.camera << "\n" <<
           "CI_q: " << (calib.CI_q).coeffs().transpose() << " C_p_I: " << calib.C_p_I.transpose() << "\n" <<
           "g: " << calib.g << "\n" <<
           "delta_t: " << calib.delta_t << "\n" <<
           "sigma_gc: " << calib.sigma_gc << " sigma_ac: " << calib.sigma_ac << "\n" <<
           "sigma_wgc: " << calib.sigma_wgc << " sigma_wac: " << calib.sigma_wac << "\n" <<
           "sigma_Im: " << calib.sigma_Im << "\n" <<
           "maxFrame: " << calib.maxFrame << "\n" <<
           "minFrame: " << calib.minFrame ;
}

