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

ros::Publisher pub_imu;      
ros::Publisher pub_image;   
ros::Publisher pub_sim_path;
ros::Publisher pub_sim_odometry;
ros::Publisher pub_sim_pose;
ros::Publisher pub_sim_cloud;   
ros::Publisher pub_odometry;
ros::Publisher pub_pose;
ros::Publisher pub_path;
tf::TransformBroadcaster br;

// visualize results
nav_msgs::Path path;
visualization_msgs::Marker path_line;
nav_msgs::Path sim_path;

Calib* calib;
DataGenerator* generator;
MSCKF* msckf;
CameraMeasurements* cameraMeasurements;

queue<sensor_msgs::Imu> imu_buf;

void imuCallback(const sensor_msgs::Imu imu_msg)
{
    imu_buf.push(imu_msg);
}

void sendIMU(const sensor_msgs::Imu imu_msg)
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

void imageCallback(const sensor_msgs::PointCloud& image_msg)
{
    double t = image_msg.header.stamp.toSec();
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
    for (int i = 0; i < (int)image_msg.points.size(); i++)
    {
        int   id = image_msg.channels[0].values[i];
        double x = image_msg.points[i].x;
        double y = image_msg.points[i].y;

        image.push_back(make_pair(id, Vector2d(x, y)));
    }
    cameraMeasurements->addFeatures(image);
    msckf->updateCamera(*cameraMeasurements);
    cout << *msckf << endl;
}


void setupROS()
{
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // define pub topics
    pub_imu          = n.advertise<sensor_msgs::Imu>("/imu_3dm_gx4/imu", 1000);
    pub_image        = n.advertise<sensor_msgs::PointCloud>("/sensors/image", 1000);

    pub_sim_path     = n.advertise<nav_msgs::Path>("/simulation/path", 100);
    pub_sim_odometry = n.advertise<nav_msgs::Odometry>("/simulation/odometry", 100);
    pub_sim_pose     = n.advertise<geometry_msgs::PoseStamped>("/simulation/pose", 100);
    pub_sim_cloud    = n.advertise<sensor_msgs::PointCloud>("/simulation/cloud", 1000);

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

    sim_path.header.frame_id = "world";
    path.header.frame_id = "world";
}

void publishIMUAndSimulation()
{
    double current_time = generator->getTime();

    //get generated data
    Vector3d position     = generator->getPosition();
    Vector3d velocity     = generator->getVelocity();
    Matrix3d rotation     = generator->getRotation();
    Quaterniond quaternion = Quaterniond(rotation);
    Vector3d linear_acceleration = generator->getIMULinearAcceleration();
    Vector3d angular_velocity = generator->getIMUAngularVelocity();

    //publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(current_time);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();
    odometry.twist.twist.linear.x = velocity(0);
    odometry.twist.twist.linear.y = velocity(1);
    odometry.twist.twist.linear.z = velocity(2);
    pub_sim_odometry.publish(odometry);

    //publish pose
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(current_time);
    pose_stamped.pose = odometry.pose.pose;
    pub_sim_pose.publish(pose_stamped);

    //publish path
    sim_path.poses.push_back(pose_stamped);
    pub_sim_path.publish(sim_path);

    //publish imu data
    sensor_msgs::Imu imu;
    imu.header.frame_id = "body";
    imu.header.stamp = ros::Time(current_time);
    imu.linear_acceleration.x = linear_acceleration(0);
    imu.linear_acceleration.y = linear_acceleration(1);
    imu.linear_acceleration.z = linear_acceleration(2);
    imu.angular_velocity.x = angular_velocity(0);
    imu.angular_velocity.y = angular_velocity(1);
    imu.angular_velocity.z = angular_velocity(2);
    imu.orientation.x = quaternion.x();
    imu.orientation.y = quaternion.y();
    imu.orientation.z = quaternion.z();
    imu.orientation.w = quaternion.w();
    pub_imu.publish(imu);
    imuCallback(imu);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(position(0), position(1), position(2)));
    tf::Quaternion q;
    q.setW(quaternion.w());
    q.setX(quaternion.x());
    q.setY(quaternion.y());
    q.setZ(quaternion.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time();
    sensor_msgs::ChannelFloat32 ids;
    int i = 0;
    for (auto & it : generator->getCloud())
    {
        geometry_msgs::Point32 p;
        p.x = it(0);
        p.y = it(1);
        p.z = it(2);
        point_cloud.points.push_back(p);
        ids.values.push_back(i++);
    }
    point_cloud.channels.push_back(ids);
    pub_sim_cloud.publish(point_cloud);
}

void publishImageData() 
{
    sensor_msgs::PointCloud feature;
    sensor_msgs::ChannelFloat32 ids;
    sensor_msgs::ChannelFloat32 pixel;
    for (int i = 0; i < generator->ROW; i++)
        for (int j = 0; j < generator->COL; j++)
            pixel.values.push_back(255);
    feature.header.stamp = ros::Time(generator->getTime());

    for (auto & id_pts : generator->getImage())
    {
        int id = id_pts.first;
        geometry_msgs::Point32 p;
        p.x = id_pts.second(0);
        p.y = id_pts.second(1);
        p.z = 1;

        feature.points.push_back(p);
        ids.values.push_back(id);
    }
    feature.channels.push_back(ids);
    feature.channels.push_back(pixel);

    pub_image.publish(feature);
    imageCallback(feature);
}

void setup()
{
    calib = new Calib();
    calib->camera.setIntrinsicParam(365.07984, 365.12127, 381.0196, 254.4431);
    calib->camera.setDistortionParam(-2.842958e-1, 8.7155025e-2, -1.4602925e-4, -6.149638e-4, -1.218237e-2);

    calib->delta_t = (double) 1 / DataGenerator::FREQ;
    calib->CI_q = Quaterniond((Matrix3d() << 0, -1, 0, 0, 0, 1, -1, 0, 0).finished());
    calib->C_p_I << -0.02, -0.14, 0; 
    calib->image_imu_offset_t = 0;

    calib->sigma_gc = 0;
    calib->sigma_ac = 0;
    calib->sigma_wgc = 0;
    calib->sigma_wac = 0; 

    calib->sigma_Im = 0;

    calib->maxFrame = ODO_MAX_FRAMES;
    calib->minFrame = 0;

    generator = new DataGenerator(calib);
    msckf = new MSCKF(calib);
    msckf->x.segment<4>(0) = Quaterniond(generator->getRotation()).coeffs();
    msckf->x.segment<3>(0 + 4) = generator->getPosition();
    msckf->x.segment<3>(0 + 4 + 3) = generator->getVelocity();
    msckf->I_a_dly = generator->getIMULinearAcceleration();
    msckf->I_g_dly = generator->getIMUAngularVelocity();

    cameraMeasurements = new CameraMeasurements();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msckf_vins");

    setupROS();
    setup();

    ros::Rate loop_rate(generator->FREQ);
    for (int publish_count = 0; ros::ok(); publish_count++)
    {
        publishIMUAndSimulation();

        //publish image data
        if (publish_count % generator->IMU_PER_IMG == 0)
        {
            publishImageData();
            cout << "simulation p: " << generator->getPosition().transpose() << endl;
            cout << "simulation v: " << generator->getVelocity().transpose() << endl;
            cout << "simulation q: " << Quaterniond(generator->getRotation()).coeffs().transpose() << endl;

        }
        publishOdometryResult();

        //update work
        generator->update();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


