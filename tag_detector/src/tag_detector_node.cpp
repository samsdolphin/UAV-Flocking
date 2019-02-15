#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/posetracker.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>

using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;

MarkerDetector MDetector;
MarkerMap MarkerMapConfig;
float MarkerSize = 0.12;
//float MarkerWithMargin = 0.318;
ros::Publisher pub_odom;
cv::Mat K, D;

void process(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            R_ref(i,j) = r.at<double>(i, j);
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom.publish(odom_ref);
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
	double p_x = 0.0;
	double p_y = 0.0;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 0 || nth == 1) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    try
    {
        Mat InImage = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        vector<cv::Point3f> pts_3;
        vector<cv::Point2f> pts_2;

        for(auto m:MDetector.detect(InImage))
        {
            int idx = MarkerMapConfig.getIndexOfMarkerId(m.id);

            char str[100];
            for (int i=0; i<4; i++)
            {
                pts_3.push_back(getPositionFromIndex(idx, i));
                pts_2.push_back(m[i]);
                sprintf(str, "%d", i);
                cv::putText(InImage, str, m[i], CV_FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0,0,255,255));
            }
            m.draw(InImage);
        }

        if (pts_3.size() >= 4)
            process(pts_3, pts_2, img_msg->header.stamp);

        imshow("view", InImage);
        waitKey(10);
    }
    catch(cv_bridge::Exception& ex)
    {
        ROS_ERROR("'%s'", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detecotr");
    ros::NodeHandle nh("~");
    MDetector.setDictionary("ARUCO_MIP_36h12");
    namedWindow("view", 1); // 1:autosize, 0:resize
    //resizeWindow("view", 640, 360);

    ros::Subscriber img_sub = nh.subscribe("image_raw", 100, img_callback);
    pub_odom = nh.advertise<nav_msgs::Odometry>("tag_odom", 10);

    MarkerMapConfig.readFromFile("/home/nvidia/catkin_ws/src/tag_detector/config/outconfig.yml");
	K = (Mat_<double>(3,3)<<932.247529, 0.000000, 635.466246, 0.000000, 932.448344, 447.605581, 0.000000, 0.000000, 1.000000);
	D = (Mat_<double>(1,5)<<-0.391250, 0.120670, -0.001217, -0.000112, 0.000000);

    ros::spin();
    destroyWindow("view");
}
