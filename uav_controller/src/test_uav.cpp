#include <iostream>
#include <utility> // std::pair
#include <cmath>
#include <vector>
#include <list>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

ros::Publisher follower_path_pub;
ros::Publisher leader_path_pub;

ros::Subscriber body_pose_sub;
ros::Subscriber marker_pose_sub;

visualization_msgs::Marker leader_path;
visualization_msgs::Marker follower_path;

Vector3d follower_pose, leader_pose(0,0,0);

void body_pose_callback(const nav_msgs::Odometry &msg)
{
	follower_pose << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
}

void marker_pose_callback(const geometry_msgs::PoseStampedConstPtr &cmsg)
{
	Vector3d cur;
	cur << cmsg->pose.position.x, cmsg->pose.position.y, cmsg->pose.position.z;
	if(leader_pose.norm() == 0)
		leader_pose = cur;
	if((cur - leader_pose).norm() < 2)
	{
		leader_pose = cur;
		geometry_msgs::Point leader;
	    follower_path.header.stamp = cmsg->header.stamp;
	    leader.x = cmsg->pose.position.x;
	    leader.y = cmsg->pose.position.y;
	    leader.z = cmsg->pose.position.z;
	    follower_path.points.push_back(leader);
	    follower_path.header.frame_id = "world";
	    follower_path.action = visualization_msgs::Marker::ADD;
	    follower_path.pose.orientation.w = 1.0;
	    follower_path.id = 2;
	    follower_path.type = visualization_msgs::Marker::LINE_STRIP;
	    follower_path.scale.x = 0.01;
	    follower_path.color.r = follower_path.color.a = 1.0;
	    leader_path_pub.publish(follower_path);

	    geometry_msgs::Point follower;
	    leader_path.header.stamp = cmsg->header.stamp;
	    follower.x = follower_pose(0);
	    follower.y = follower_pose(1);
	    follower.z = follower_pose(2);
	    leader_path.points.push_back(follower);
	    leader_path.header.frame_id = "world";
	    leader_path.action = visualization_msgs::Marker::ADD;
	    leader_path.pose.orientation.w = 1.0;
	    leader_path.id = 1;
	    leader_path.type = visualization_msgs::Marker::LINE_STRIP;
	    leader_path.scale.x = 0.01;
	    leader_path.color.b = leader_path.color.a = 1.0;
	    follower_path_pub.publish(leader_path);
	}	
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_uav");
    ros::NodeHandle nh("~");

    body_pose_sub = nh.subscribe("/vins_estimator/imu_propagate", 100, body_pose_callback);
    marker_pose_sub = nh.subscribe("/marker_detector/marker_pose", 100, marker_pose_callback);

    follower_path_pub= nh.advertise<visualization_msgs::Marker>("follower", 1);
    leader_path_pub= nh.advertise<visualization_msgs::Marker>("leader", 1);

    ros::spin();
    return 0;
}