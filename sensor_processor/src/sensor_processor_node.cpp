#include <cstdlib>
#include <vector>
using namespace std;

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

Mat K, D, map1, map2, map1_fixed, map2_fixed;
const int MAX_CNT = 50;
const int MIN_DIST = 30, HASH_DIST = 2 * MIN_DIST;
const int ROW = 480, HASH_ROW = (ROW + HASH_DIST - 1) / HASH_DIST + 1;
const int COL = 752, HASH_COL = (COL + HASH_DIST - 1) / HASH_DIST + 1;
const double FREQ_TIME = 0.1;

bool img_hash[HASH_COL][HASH_ROW];

Mat prev_img, cur_img, forw_img;
vector<Point2f> prev_pts, cur_pts, forw_pts;
double prev_time, cur_time, forw_time;

int next_id = 0;
vector<int> id;

ros::Publisher pub_image;

template<typename T>
void reduce_vector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

double sum_time = 0.0;
int sum_cnt = 0;

void image_callback(const sensor_msgs::ImagePtr &image_msg)
{
    forw_time = image_msg->header.stamp.toSec();
    ROS_DEBUG("current time %lf", forw_time);

    double t_s = clock();

    Mat dist_img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8)->image;
    forw_img.release();
    //undistort(dist_img, forw_img, K, D);
    remap(dist_img, forw_img, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
    ROS_DEBUG("read and undistort costs %lf", (clock() - t_s) / CLOCKS_PER_SEC * 1000);

    if (prev_img.empty())
    {
        ROS_DEBUG("init");

        prev_img = forw_img;
        goodFeaturesToTrack(prev_img, prev_pts, MAX_CNT, 0.05, MIN_DIST);
        prev_time = forw_time;

        for (int i = 0; i < int(prev_pts.size()); i++)
            id.push_back(next_id++);

        ROS_DEBUG("feature number: %d", int(prev_pts.size()));

        cur_img = prev_img;
        cur_pts = prev_pts;
        cur_time = prev_time;

        ROS_DEBUG("sum time %lf", sum_time);
        sum_time = 0.0;
        return;
    }


    double t_op = clock();
    ROS_DEBUG("start tracking");

    vector<uchar> status;
    vector<float> err;
    forw_pts.clear();

    ROS_DEBUG("tracking number: %lu", cur_pts.size());
    calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, Size(21, 21), 3);
    reduce_vector(prev_pts, status);
    reduce_vector(cur_pts, status);
    reduce_vector(forw_pts, status);
    reduce_vector(id, status);
    ROS_DEBUG("tracking number: %lu", forw_pts.size());
    ROS_DEBUG("optical flow costs %lf", (clock() - t_op) / CLOCKS_PER_SEC * 1000);

    if (forw_time - prev_time < FREQ_TIME)
    {
        cv::swap(cur_img, forw_img);
        std::swap(cur_pts, forw_pts);
        std::swap(cur_time, forw_time);
        sum_time += (clock() - t_s) / CLOCKS_PER_SEC * 1000.0;
    }
    else
    {
        vector<uchar> status;
        vector<float> err;

        if (prev_pts.size() >= 9)
        {
            double t_f = clock();
            findFundamentalMat(prev_pts, forw_pts, FM_RANSAC, 0.5, 0.99, status);
            reduce_vector(prev_pts, status);
            reduce_vector(forw_pts, status);
            reduce_vector(id, status);
            ROS_DEBUG("F costs %lf", (clock() - t_f) / CLOCKS_PER_SEC * 1000);
        }

        memset(img_hash, 0, sizeof(img_hash));
        for (auto & i : forw_pts)
        {
            int x = int(i.x / HASH_DIST + 0.5);
            int y = int(i.y / HASH_DIST + 0.5);
            if (x >= 0 && x < HASH_COL && y >= 0 && y < HASH_ROW)
                img_hash[x][y] = true;
            else
                ROS_ERROR("Point(%lf %lf), (%d, %d) is out of (%d, %d)", i.x, i.y, x, y, HASH_COL, HASH_ROW);
        }

        double t_g = clock();
        vector<Point2f> tmp_pts;
        goodFeaturesToTrack(forw_img, tmp_pts, MAX_CNT * 2, 0.05, MIN_DIST);
        ROS_DEBUG("tracking new feature costs %lf", (clock() - t_g) / CLOCKS_PER_SEC * 1000);

        int cnt = 0;
        for (auto & i : tmp_pts)
        {
            int x = int(i.x / HASH_DIST + 0.5);
            int y = int(i.y / HASH_DIST + 0.5);
            if (x >= 0 && x < HASH_COL && y >= 0 && y < HASH_ROW && !img_hash[x][y])
            {
                forw_pts.push_back(i);
                id.push_back(next_id++);
                cnt++;
            }
        }

        sum_time += (clock() - t_s) / CLOCKS_PER_SEC * 1000.0;
        sum_cnt++;
        ROS_DEBUG("sum time %lf", sum_time);
        ROS_DEBUG("sum time %lf", sum_time / sum_cnt);

        sensor_msgs::PointCloud feature;
        sensor_msgs::ChannelFloat32 ids;
        sensor_msgs::ChannelFloat32 pixel;

        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                pixel.values.push_back(forw_img.at<uchar>(i, j));

        feature.header = image_msg->header;
        vector<Point2f> un_pts;
        undistortPoints(forw_pts, un_pts, K, Mat());
        for (int i = 0; i < int(un_pts.size()); i++)
        {
            int p_id = id[i];
            geometry_msgs::Point32 p;
            p.x = un_pts[i].x;
            p.y = un_pts[i].y;
            p.z = 1;

            feature.points.push_back(p);
            ids.values.push_back(p_id);
        }
        feature.channels.push_back(ids);
        feature.channels.push_back(pixel);
        pub_image.publish(feature);

        Mat color_img;
        cvtColor(forw_img, color_img, CV_GRAY2RGB);
        for (int i = 0; i < int(prev_pts.size()); i++)
            line(color_img, forw_pts[i], prev_pts[i], Scalar(0, 255, 0), 3);
        for (int i = 0; i < int(forw_pts.size()); i++)
            circle(color_img, forw_pts[i], 3, Scalar(255, 0, 0));

        imshow("tracking", color_img);
        waitKey(1);

        cv::swap(prev_img, forw_img);
        std::swap(prev_pts, forw_pts);
        std::swap(prev_time, forw_time);

        cur_img = prev_img;
        cur_pts = prev_pts;
        cur_time = prev_time;
    }
    puts("");
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_processor");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::Subscriber sub_image = n.subscribe("input_image", 1000, image_callback);


    XmlRpc::XmlRpcValue xml_value;
    float _K[3 * 3], _D[8];
    if (n.getParam("K", xml_value))
    {
        for (int i = 0; i < xml_value.size(); i++)
            _K[i] = static_cast<double>(xml_value[i]);
        K = Mat(3, 3, CV_32FC1, _K);
        ROS_INFO_STREAM("K: " << K);
    }
    else
    {
        ROS_ERROR("Error K");
    }

    if (n.getParam("D", xml_value))
    {
        for (int i = 0; i < xml_value.size(); i++)
            _D[i] = static_cast<double>(xml_value[i]);
        D = Mat(1, xml_value.size(), CV_32FC1, _D);
        ROS_INFO_STREAM("D: " << D);
    }
    else
    {
        ROS_ERROR("Error D");
    }

    initUndistortRectifyMap(K, D, Mat(), Mat(), Size(COL, ROW), CV_32FC1, map1, map2);
    convertMaps(map1, map2, map1_fixed, map2_fixed, CV_16SC2);

    pub_image = n.advertise<sensor_msgs::PointCloud>("output_image", 1000);

    ros::spin();

    return 0;
}
