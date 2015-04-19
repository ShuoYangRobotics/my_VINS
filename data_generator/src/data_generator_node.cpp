#include "ros/ros.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"

#include "data_generator.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace Eigen;


#define ROW 480
#define COL 752


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_generator");
    ros::NodeHandle n;

    ros::Publisher pub_imu      = n.advertise<sensor_msgs::Imu>("/imu_3dm_gx4/imu", 1000);
    ros::Publisher pub_image    = n.advertise<sensor_msgs::PointCloud>("/sensors/image", 1000);

    ros::Publisher pub_path     = n.advertise<nav_msgs::Path>("/simulation/path", 100);
    ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("/simulation/odometry", 100);
    ros::Publisher pub_pose     = n.advertise<geometry_msgs::PoseStamped>("/simulation/pose", 100);
    ros::Publisher pub_cloud    = n.advertise<sensor_msgs::PointCloud>("/simulation/cloud", 1000);

    DataGenerator generator;
    ros::Rate loop_rate(generator.FREQ);

    //while (pub_imu.getNumSubscribers() == 0 || pub_image.getNumSubscribers() == 0)
    //    loop_rate.sleep();

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time();
    for (auto & it : generator.getCloud())
    {
        geometry_msgs::Point32 p;
        p.x = it(0);
        p.y = it(1);
        p.z = it(2);
        point_cloud.points.push_back(p);
    }
    pub_cloud.publish(point_cloud);

    cv::namedWindow("camera image", cv::WINDOW_AUTOSIZE);

    int publish_count = 0;

    nav_msgs::Path path;
    path.header.frame_id = "world";

    while (ros::ok())
    {
        double current_time = generator.getTime();
        ROS_INFO("time: %lf", current_time);

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

        pub_imu.publish(imu);
        ROS_INFO("publish imu data with stamp %lf", imu.header.stamp.toSec());

        pub_cloud.publish(point_cloud);
        //publish image data
        if (publish_count % generator.IMU_PER_IMG == 0)
        {
            //publish image data
            ROS_INFO("feature count: %lu", generator.getImage().size());

            sensor_msgs::PointCloud feature;
            sensor_msgs::ChannelFloat32 ids;
            sensor_msgs::ChannelFloat32 pixel;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    pixel.values.push_back(255);
            feature.header.stamp = ros::Time(generator.getTime());
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
                cv::putText(simu_img, label, cv::Point2d(p.x + 1, p.y + 1) * 0.5 * 600, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
            }
            feature.channels.push_back(ids);
            feature.channels.push_back(pixel);
            pub_image.publish(feature);
            ROS_INFO("publish image data with stamp %lf", feature.header.stamp.toSec());
            cv::imshow("camera image", simu_img);
            cv::waitKey(100);
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
