#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <queue>
#include "data_generator.h"
#include "camera.h"
#include "calib.h"
#include "odometry.h"

using namespace std;

ros::Publisher pub_odometry;
ros::Publisher pub_pose;
ros::Publisher pub_path;
ros::Subscriber sub_imu;
ros::Subscriber sub_image;

// visualize results
nav_msgs::Path path;
visualization_msgs::Marker path_line;

Calib* calib;
MSCKF* msckf;
CameraMeasurements* cameraMeasurements;

queue<sensor_msgs::Imu> imu_buf;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    ROS_INFO("received imu data with stamp %lf", imu_msg->header.stamp.toSec());

    imu_buf.push(*imu_msg);
}

void sendIMU(const sensor_msgs::Imu& imu_msg)
{
    double t = imu_msg.header.stamp.toSec();

    ROS_INFO("processing imu data with stamp %lf", t);

    double dx = imu_msg.linear_acceleration.x;
    double dy = imu_msg.linear_acceleration.y;
    double dz = imu_msg.linear_acceleration.z;

    double rx = imu_msg.angular_velocity.x;
    double ry = imu_msg.angular_velocity.y;
    double rz = imu_msg.angular_velocity.z;

    msckf->propagate(Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void publishOdometryResult()
{
    Vector3d p = msckf->getPosition();
    Vector3d v = msckf->getVelocity();
    Quaterniond q = msckf->getQuaternion();

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = p(0);
    odometry.pose.pose.position.y = p(1);
    odometry.pose.pose.position.z = p(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    odometry.twist.twist.linear.x = v(0);
    odometry.twist.twist.linear.y = v(1);
    odometry.twist.twist.linear.y = v(2);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;

    path.poses.push_back(pose_stamped);

    pub_odometry.publish(odometry);
    pub_pose.publish(pose_stamped);
    pub_path.publish(path);
}

void imageCallback(const sensor_msgs::PointCloud::ConstPtr& image_msg_ptr)
{
    double t = image_msg_ptr->header.stamp.toSec();
    if (imu_buf.empty() || t < imu_buf.front().header.stamp.toSec())
    {
        ROS_ERROR("wait for imu data");
        return;
    }

    while (!imu_buf.empty() && t >= imu_buf.front().header.stamp.toSec())
    {
        sendIMU(imu_buf.front());
        imu_buf.pop();
    }

    ROS_INFO("processing vision data with stamp %lf", t);

    vector<pair<int, Vector2d> > image;
    for (int i = 0; i < (int) image_msg_ptr->points.size(); i++)
    {
        int   id = image_msg_ptr->channels[0].values[i];
        double x = image_msg_ptr->points[i].x;
        double y = image_msg_ptr->points[i].y;

        image.push_back(make_pair(id, Vector2d(x, y)));
    }
    cameraMeasurements->addFeatures(image);
    msckf->updateCamera(*cameraMeasurements);
    //cout << *msckf << endl;

    publishOdometryResult();
}

void setupROS()
{
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // define pub topics
    pub_path         = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry     = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_pose         = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);

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

    path.header.frame_id = "world";

    sub_imu = n.subscribe("/imu_3dm_gx4/imu", 1000, imuCallback);
    sub_image = n.subscribe("/sensors/image", 1000, imageCallback);
}

void setup()
{
    calib = new Calib();
    calib->camera.setIntrinsicParam(365.07984, 365.12127, 381.01962, 254.44318);
    calib->camera.setDistortionParam(-2.842958e-1, 8.7155025e-2, -1.4602925e-4, -6.149638e-4, -1.218237e-2);

    calib->delta_t = (double) 1 / DataGenerator::FREQ;
    calib->CI_q = Quaterniond((Matrix3d() << 0, -1, 0, 0, 0, 1, -1, 0, 0).finished());
    calib->C_p_I << -0.02, -0.14, 0; 
    calib->image_imu_offset_t = 0;

    calib->sigma_gc = 0.01;
    calib->sigma_ac = 0.1;
    calib->sigma_wgc = 0.01;
    calib->sigma_wac = 0.01; 
    calib->sigma_Im = 10;
    
    DataGenerator* generator = new DataGenerator(calib);

    msckf = new MSCKF(calib);
    msckf->x.segment<4>(0) = Quaterniond(generator->getRotation()).coeffs();
    msckf->x.segment<3>(0 + 4) = generator->getPosition();
    msckf->x.segment<3>(0 + 4 + 3) = generator->getVelocity();

    cameraMeasurements = new CameraMeasurements();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msckf_vins");

    setup();
    setupROS();
    ros::spin();

    return 0;
}


