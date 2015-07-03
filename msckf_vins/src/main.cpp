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

#include "data_generator.h"
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
Quaterniond curr_q;
visualization_msgs::Marker path_line;
ros::Publisher pub_odometry;
ros::Publisher pub_path2, pub_path3, pub_path4;
ros::Publisher pub_pose2, pub_pose3;

void imu_callback(const sensor_msgs::Imu imu_msg)
{
    imu_buf.push(imu_msg);
}

void send_imu(const sensor_msgs::Imu imu_msg)
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

void image_callback(const sensor_msgs::PointCloud image_msg)
{
    my_kf.setErrorCovarianceIdentity();
    double t = image_msg.header.stamp.toSec();
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
    for (int i = 0; i < (int)image_msg.points.size(); i++)
    {
        int   id = image_msg.channels[0].values[i];
        double x = image_msg.points[i].x;
        double y = image_msg.points[i].y;
        double z = image_msg.points[i].z;
        Vector3d p_cf(x, y, z);
        Vector2d cam_ptr = my_kf.projectCamPoint(p_cf);
        //ROS_INFO("id %d cam pos (%f, %f, %f) project to (%f, %f)", id, x, y, z, cam_ptr(0), cam_ptr(1));
        if(cam_ptr(0) > 0 && cam_ptr(0) < my_kf.cam.width && cam_ptr(1) > 0 && cam_ptr(1) < my_kf.cam.height) 
          //image.push_back(make_pair(/*gr_id * 10000 + */id, Vector3d(cam_ptr(0), cam_ptr(1), 1)));
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
    odometry.pose.pose.orientation.x = curr_q.x();
    odometry.pose.pose.orientation.y = curr_q.y();
    odometry.pose.pose.orientation.z = curr_q.z();
    odometry.pose.pose.orientation.w = curr_q.w();
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
    pub_path2.publish(path);
    pub_pose2.publish(pose_stamped);

    geometry_msgs::Point pose_p;
    pose_p.x = last_path(0);
    pose_p.y = last_path(1);
    pose_p.z = last_path(2);

    path_line.points.push_back(pose_p);
    path_line.scale.x = 0.01;
    pub_path3.publish(path_line);
    path_line.scale.x = 0.5;
    pub_path4.publish(path_line);

    // broadcast tf

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //transform.setOrigin(tf::Vector3(last_path(0), last_path(1), last_path(2)) );
    tf::Quaternion q;
    q.setW(curr_q.w());
    q.setX(curr_q.x());
    q.setY(curr_q.y());
    q.setZ(curr_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));

//    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//    q.setEuler(0.0, M_PI, 0.0);
//    transform.setRotation(q);
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world_v"));
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "body_v"));

}

void cloud_callback(const sensor_msgs::PointCloud cloud_msg)
{

    for (int i = 0; i < (int)cloud_msg.points.size(); i++)
    {
        int   id = cloud_msg.channels[0].values[i];
        double x = cloud_msg.points[i].x;
        double y = cloud_msg.points[i].y;
        double z = cloud_msg.points[i].z;
        Vector3d world_ptr(x, y, z);
        my_kf.global_features[id] = world_ptr;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msckf_vins");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // define pub topics
    ros::Publisher pub_imu      = n.advertise<sensor_msgs::Imu>("/imu_3dm_gx4/imu", 1000);
    ros::Publisher pub_image    = n.advertise<sensor_msgs::PointCloud>("/sensors/image", 1000);

    ros::Publisher pub_path     = n.advertise<nav_msgs::Path>("/simulation/path", 100);
    ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("/simulation/odometry", 100);
    ros::Publisher pub_pose     = n.advertise<geometry_msgs::PoseStamped>("/simulation/pose", 100);
    ros::Publisher pub_cloud    = n.advertise<sensor_msgs::PointCloud>("/simulation/cloud", 1000);
    pub_path2     = n.advertise<nav_msgs::Path>("path2", 1000);
    pub_path3    = n.advertise<visualization_msgs::Marker>("path3", 1000);
    pub_path4    = n.advertise<visualization_msgs::Marker>("path4", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_pose2     = n.advertise<geometry_msgs::PoseStamped>("pose2", 1000);
    pub_pose3    = n.advertise<geometry_msgs::PoseStamped>("pose3", 1000);

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
    pub_path2.publish(path_line);
    path_line.scale.x            = 0.5;
    pub_path3.publish(path_line);

    path.header.frame_id = "world";

    // init MSCKF
    Vector4d init_q(0.0, 0.0, 0.0, 1.0);  // x y z w
    Vector3d init_p(10.0, 0.0, 3.0);
    Vector3d init_v(0.0, 1.0, 0.0);
    Vector3d init_bg(0.0 ,0.0, 0.0);
    Vector3d init_ba(0.0 ,0.0, 0.0);
    Vector3d init_pcb(-0.14, -0.02, 0.0);
    my_kf.setCalibParam(init_pcb, 365.07984, 365.12127, 381.0196, 254.4431,
                            -2.842958e-1, 8.7155025e-2, -1.4602925e-4, -6.149638e-4, -1.218237e-2);
    my_kf.setNominalState(init_q, init_p, init_v, init_bg, init_ba);
    my_kf.setMeasureNoise(0.0);
    my_kf.setNoiseMatrix(0.0, 0.0, 0.0, 0.0);

    DataGenerator generator;
    ros::Rate loop_rate(generator.FREQ);
    int publish_count = 0;
    nav_msgs::Path path;
    path.header.frame_id = "world";    
    //prepare point cloud 
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time();
    sensor_msgs::ChannelFloat32 ids;
    int i = 0;
    for (auto & it : generator.getCloud())
    {
        geometry_msgs::Point32 p;
        p.x = it(0);
        p.y = it(1);
        p.z = it(2);
        point_cloud.points.push_back(p);
        ids.values.push_back(i++);
    }
    point_cloud.channels.push_back(ids);
    pub_cloud.publish(point_cloud);

    while (ros::ok())
    {
        double current_time = generator.getTime();
        //ROS_INFO("time: %lf", current_time);

        //get generated data
        Vector3d position     = generator.getPosition();
        Vector3d velocity     = generator.getVelocity();
        Matrix3d rotation     = generator.getRotation();
        Quaterniond q(rotation);

        Vector3d linear_acceleration = generator.getLinearAcceleration();
        Vector3d angular_velocity = generator.getAngularVelocity();

        //publish visulization data
        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "world";
        odometry.header.stamp = ros::Time(current_time);
        odometry.pose.pose.position.x = position(0);
        odometry.pose.pose.position.y = position(1);
        odometry.pose.pose.position.z = position(2);
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();
        odometry.twist.twist.linear.x = velocity(0);
        odometry.twist.twist.linear.y = velocity(1);
        odometry.twist.twist.linear.z = velocity(2);
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = ros::Time(current_time);
        pose_stamped.pose = odometry.pose.pose;
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        pub_pose.publish(pose_stamped);

        //publish imu data
        sensor_msgs::Imu imu;
        imu.header.frame_id = "world";
        imu.header.stamp = ros::Time(current_time);
        imu.linear_acceleration.x = linear_acceleration(0);
        imu.linear_acceleration.y = linear_acceleration(1);
        imu.linear_acceleration.z = linear_acceleration(2);
        imu.angular_velocity.x = angular_velocity(0);
        imu.angular_velocity.y = angular_velocity(1);
        imu.angular_velocity.z = angular_velocity(2);
        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();
        imu_callback(imu);
        pub_imu.publish(imu);
        //printf("publish imu data with stamp %lf\n", imu.header.stamp.toSec());
        cloud_callback(point_cloud);
        pub_cloud.publish(point_cloud);
        
        //publish image data
        if (publish_count % generator.IMU_PER_IMG == 0)
        {
            //publish image data

            sensor_msgs::PointCloud feature;
            sensor_msgs::ChannelFloat32 ids;
            sensor_msgs::ChannelFloat32 pixel;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    pixel.values.push_back(255);
            feature.header.stamp = ros::Time(generator.getTime());

            // printf("publish image data with stamp %lf\n", feature.header.stamp.toSec());
            // printf("p_gb: [%f, %f, %f], R_gb: [%f, %f, %f, %f]\n", position(0), position(1), position(2),
            //          q.x(), q.y(), q.z(), q.w());

            cv::Mat simu_img(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
            for (auto & id_pts : generator.getImage())
            {
                int id = id_pts.first;
                geometry_msgs::Point32 p;
                p.x = id_pts.second(0);
                p.y = id_pts.second(1);
                p.z = id_pts.second(2);

                //project point
                //p.y /= p.x;
                //p.z /= p.x;
                //p.x = 1;

                feature.points.push_back(p);
                ids.values.push_back(id);

                p.x /= p.z;
                p.y /= p.z;

                char label[10];
                sprintf(label, "%d", id);
                cv::putText(simu_img, label, cv::Point2d(p.y + 1, p.x + 1) * 0.5 * 600, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
            }
            feature.channels.push_back(ids);
            feature.channels.push_back(pixel);
            image_callback(feature);
            pub_image.publish(feature);
            // cv::imshow("camera image", simu_img);
            // cv::waitKey(100);
            //if (generator.getTime() > DataGenerator::MAX_TIME)
            //    break;
        }

        //update work
        generator.update();
        publish_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


