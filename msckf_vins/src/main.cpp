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
#include "math_tool.h"
using namespace std;

const int ROW=480;
const int COL=752;
const double FOCAL_LENGTH = 365.1;


queue<sensor_msgs::Imu> imu_buf;
    
MSCKF my_kf;

// visualize results
nav_msgs::Path path;
double sum_of_path = 0.0;
Vector3d last_path(0.0, 0.0, 0.0);
Vector4d curr_q;
visualization_msgs::Marker path_line;
ros::Publisher pub_odometry;
ros::Publisher pub_path, pub_path1, pub_path2;
ros::Publisher pub_pose, pub_pose2;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    imu_buf.push(*imu_msg);

//    double dx = imu_msg->linear_acceleration.x;
//    double dy = imu_msg->linear_acceleration.y;
//    double dz = imu_msg->linear_acceleration.z;
//
//    double rx = imu_msg->angular_velocity.x;
//    double ry = imu_msg->angular_velocity.y;
//    double rz = imu_msg->angular_velocity.z;
//
//    my_kf.processIMU(t, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}


void send_imu(const sensor_msgs::Imu &imu_msg)
{
    double t = imu_msg.header.stamp.toSec();

    //ROS_INFO("processing IMU data with stamp %lf", t);

    double dx = imu_msg.linear_acceleration.x;
    double dy = imu_msg.linear_acceleration.y;
    double dz = imu_msg.linear_acceleration.z;

    double rx = imu_msg.angular_velocity.x;
    double ry = imu_msg.angular_velocity.y;
    double rz = imu_msg.angular_velocity.z;

    my_kf.processIMU(t, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void image_callback(const sensor_msgs::PointCloudConstPtr &image_msg)
{
    double t = image_msg->header.stamp.toSec();
    if (imu_buf.empty() || t < imu_buf.front().header.stamp.toSec())
    {
        ROS_ERROR("wait for imu data");
        return;
    }
    TicToc t_s;
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
        Vector3d world_ptr(x, y, z);
        Vector2d cam_ptr = my_kf.projectWorldPoint(world_ptr);
        image.push_back(make_pair(/*gr_id * 10000 + */id, Vector3d(cam_ptr(0), cam_ptr(1), 1)));
    }

    my_kf.processImage(image);

    sum_of_path += (my_kf.getPosition() - last_path).norm();
    last_path = my_kf.getPosition();
    //Matrix3d Rota = my_kf.getRotation();
    //curr_q = R_to_quaternion(Rota.transpose());
    //cout << curr_q << endl; 
    //cout << curr_q.norm() << endl; 
    curr_q = my_kf.getQuaternion();

    ROS_INFO("sum of path %lf", sum_of_path);
    ROS_INFO("vo solver costs: %lf ms", t_s.toc());

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = last_path(0);
    odometry.pose.pose.position.y = last_path(1);
    odometry.pose.pose.position.z = last_path(2);
    odometry.pose.pose.orientation.x = curr_q(1);
    odometry.pose.pose.orientation.y = curr_q(2);
    odometry.pose.pose.orientation.z = curr_q(3);
    odometry.pose.pose.orientation.w = curr_q(0);
//    odometry.twist.twist.linear.x = solution.v(0);
//    odometry.twist.twist.linear.y = solution.v(1);
//    odometry.twist.twist.linear.z = solution.v(2);
    pub_odometry.publish(odometry);

//fprintf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", solution.p(0), solution.p(1), solution.p(2),
//        solution.v(0), solution.v(1), solution.v(2),
//        solution.q.x(), solution.q.y(), solution.q.z(), solution.q.w());


    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
    pub_pose.publish(pose_stamped);

    geometry_msgs::Point pose_p;
    pose_p.x = last_path(0);
    pose_p.y = last_path(1);
    pose_p.z = last_path(2);

    path_line.points.push_back(pose_p);
    path_line.scale.x = 0.01;
    pub_path1.publish(path_line);
    path_line.scale.x = 0.5;
    pub_path2.publish(path_line);

    // broadcast tf

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //transform.setOrigin(tf::Vector3(last_path(0), last_path(1), last_path(2)) );
    tf::Quaternion q;
    q.setW(curr_q(0));
    q.setX(curr_q(1));
    q.setY(curr_q(2));
    q.setZ(curr_q(3));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));

//    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//    q.setEuler(0.0, M_PI, 0.0);
//    transform.setRotation(q);
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world_v"));
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "body_v"));

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msckf_vins");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // define pub topics
    pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    pub_path1    = n.advertise<visualization_msgs::Marker>("path1", 1000);
    pub_path2    = n.advertise<visualization_msgs::Marker>("path2", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_pose     = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    pub_pose2    = n.advertise<geometry_msgs::PoseStamped>("pose2", 1000);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

//    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//    q.setEuler(0.0, M_PI, 0.0);
//    transform.setRotation(q);
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world_v"));
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "body_v"));


    path_line.header.frame_id    = "world";
    path_line.header.stamp       = ros::Time::now();
    path_line.ns                 = "calibration";
    path_line.action             = visualization_msgs::Marker::ADD;
    path_line.pose.orientation.w = 1.0;
    path_line.type               = visualization_msgs::Marker::LINE_STRIP;
    path_line.scale.x            = 0.01;
    path_line.color.a            = 1.0;
    path_line.color.r            = 1.0;
    path_line.id                 = 1;
    path_line.points.push_back(geometry_msgs::Point());
    pub_path1.publish(path_line);
    path_line.scale.x            = 0.5;
    pub_path2.publish(path_line);

    path.header.frame_id = "world";

    // init MSCKF
    Vector4d init_q(1.0, 0.0, 0.0, 0.0);  // w x y z
    Vector3d init_p(0.0, 0.0, 0.0);
    Vector3d init_v(0.0, 1.0, 0.0);
    Vector3d init_bg(0.0 ,0.0, 0.0);
    Vector3d init_ba(0.0 ,0.0, 0.0);
    Vector3d init_pcb(-0.14, -0.02, 0.0);
    my_kf.setCalibParam(init_pcb, 365.07984, 365.12127, 381.0196, 254.4431,
                            -2.842958e-1, 8.7155025e-2, -1.4602925e-4, -6.149638e-4, -1.218237e-2);
    my_kf.setNominalState(init_q, init_p, init_v, init_bg, init_ba);

    ros::Subscriber sub_imu   = n.subscribe("/imu_3dm_gx4/imu", 1000, imu_callback);
    ros::Subscriber sub_image = n.subscribe("/sensors/image", 1000, image_callback);

    ros::spin();

    return 0;
}


