#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "tic_toc.h"
#include "MSCKF.h"
using namespace std;

const int ROW=480;
const int COL=752;
const double FOCAL_LENGTH = 365.1;


queue<sensor_msgs::Imu> imu_buf;
    
MSCKF my_kf;


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    imu_buf.push(*imu_msg);
}


void send_imu(const sensor_msgs::Imu &imu_msg)
{
    double t = imu_msg.header.stamp.toSec();

    ROS_INFO("processing IMU data with stamp %lf", t);

    double dx = imu_msg.linear_acceleration.x;
    double dy = imu_msg.linear_acceleration.y;
    double dz = imu_msg.linear_acceleration.z;

    double rx = imu_msg.angular_velocity.x;
    double ry = imu_msg.angular_velocity.y;
    double rz = imu_msg.angular_velocity.z;

    my_kf.processIMU(t, Vector3f(dx, dy, dz), Vector3f(rx, ry, rz));
}

void image_callback(const sensor_msgs::PointCloudConstPtr &image_msg)
{
    double t = image_msg->header.stamp.toSec();
    if (imu_buf.empty() || t < imu_buf.front().header.stamp.toSec())
    {
        ROS_ERROR("wait for imu data");
        return;
    }
    while (!imu_buf.empty() && t >= imu_buf.front().header.stamp.toSec())
    {
        send_imu(imu_buf.front());
        imu_buf.pop();
    }
    ROS_INFO("processing vision data with stamp %lf", t);
    vector<pair<int, Vector3d>> image;
    for (int i = 0; i < (int)image_msg->points.size(); i++)
    {
        int   id = image_msg->channels[0].values[i];
        double x = image_msg->points[i].x;
        double y = image_msg->points[i].y;
        double z = image_msg->points[i].z;
        image.push_back(make_pair(/*gr_id * 10000 + */id, Vector3d(x, y, z)));
    }

    TicToc t_s;
    my_kf.processImage(image);
    //vector<pair<Vector3d, Vector3d>> corres;
    //SolutionContainer solution = estimator.processImage(image, corres);

    //sum_of_path += (solution.p - last_path).norm();
    //last_path = solution.p;
    //ROS_INFO("sum of path %lf", sum_of_path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msckf_vins");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setEuler(0.0, M_PI, 0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world_v"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "body_v"));

    // init MSCKF

    ros::Subscriber sub_imu   = n.subscribe("/imu_3dm_gx4/imu", 1000, imu_callback);
    ros::Subscriber sub_image = n.subscribe("/sensors/image", 1000, image_callback);

    ros::spin();

    return 0;
}


