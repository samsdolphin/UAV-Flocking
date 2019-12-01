#include <iostream>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;
using namespace std;

ros::Publisher position_cmd_pub;

ros::Subscriber body_pose_sub;
ros::Subscriber trigger_sub;
ros::Subscriber marker_pose_sub;

#define uav_height 1.2
#define max_a 0.09
#define max_v 0.4
#define H 1.0
#define beta 0.25
#define theta 2
#define d0 1
#define d1 5

Vector3d pre_relative_p(0, 0, 0);
Vector3d tar(0, 0, 0);
Vector3d p(0, 0, uav_height);
Vector3d v(0, 0, 0);

double pre_t = 0.0;
double pre_ax = 0.0;
double pre_ay = 0.0;
double temp_ax = 0.0;
double temp_ay = 0.0;
double jerk_x = 0.0;
double jerk_y = 0.0;
double loop_t = 0.0;
double start_t = 0.0;
double delta = 0.0;

int traj_id_send = 0;
int cnt = 0;

bool is_init = false;
bool is_triggered = false;

void body_pose_callback(const nav_msgs::Odometry &msg)
{
    is_init = true;
    p << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
    v << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;

    if(is_triggered)
    {
        Vector3d position, velocity;

        double t = ros::Time::now().toSec() - loop_t;
        double accx = 0.0;
        double accy = 0.0;

        if(temp_ax + jerk_x * cnt <= pre_ax)
            accx = temp_ax + jerk_x * cnt;
        else
            accx = pre_ax;

        if(temp_ay + jerk_y * cnt <= pre_ay)
            accy = temp_ay + jerk_y * cnt;
        else
            accy = pre_ay;

        velocity(0) = v(0) + accx * t;
        velocity(1) = v(1) + accy * t;
        velocity(2) = 0;

        double vnorm = velocity.norm();
        if(vnorm > max_v)
            velocity *= max_v/vnorm;

        position(0) = p(0) + v(0) * t + 0.5 * accx * pow(t, 2);
        position(1) = p(1) + v(1) * t + 0.5 * accy * pow(t, 2);
        position(2) = uav_height;

        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = msg.header.stamp;
        cmd.header.frame_id = "world";

        cmd.position.x = position(0);
        cmd.position.y = position(1);
        cmd.position.z = position(2);

        cmd.velocity.x = velocity(0);
        cmd.velocity.y = velocity(1);
        cmd.velocity.z = velocity(2);

        cmd.acceleration.x = accx;
        cmd.acceleration.y = accy;
        cmd.acceleration.z = 0;
        
        cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id_send;

        position_cmd_pub.publish(cmd);
        loop_t = ros::Time::now().toSec();
        cnt++;
        cnt = cnt % 10;
    }
    else
    {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = msg.header.stamp;
        cmd.header.frame_id = "world";

        cmd.position.x = p(0);
        cmd.position.y = p(1);
        cmd.position.z = uav_height;

        cmd.velocity.x = 0;
        cmd.velocity.y = 0;
        cmd.velocity.z = 0;

        cmd.acceleration.x = 0;
        cmd.acceleration.y = 0;
        
        cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_EMPTY;
        cmd.trajectory_id = traj_id_send;

        position_cmd_pub.publish(cmd);
    }
}

void marker_pose_callback(const geometry_msgs::PoseStampedConstPtr &cmsg)
{
    Vector3d acceleration, relative_p, relative_v;
    tar << cmsg->pose.position.z, -cmsg->pose.position.x, uav_height;

    if(pre_relative_p.norm() > 0)
    {
        delta = ros::Time::now().toSec() - pre_t;
        relative_p << tar(0), tar(1), 0;
        cout<<"relative_p: "<<relative_p(0)<<", "<<relative_p(1)<<endl;

        double aij = H / pow((1 + pow(relative_p.norm(), 2)), beta);
        double f = 1 / pow(pow(relative_p.norm(), 2) - d0, theta);
        double g = 1 / pow(pow(relative_p.norm(), 2) - d1, theta);

        relative_v = (relative_p - pre_relative_p)/delta;
        double lambda = pow(0.5 * pow(relative_v.norm(), 2), 0.5);
        pre_relative_p = relative_p;
        pre_t = ros::Time::now().toSec();

        acceleration(0) = aij * relative_v(0) - lambda * f * relative_p(0) + lambda * g * relative_p(0);
        acceleration(1) = aij * relative_v(1) - lambda * f * relative_p(1) + lambda * g * relative_p(1);
        acceleration(2) = 0;
        cout<<"des_a: "<<acceleration(0)<<", "<<acceleration(1)<<endl;

        if((acceleration(0) - pre_ax) > max_a)
        {
            jerk_x = max_a / 10;
            temp_ax = pre_ax;
            pre_ax += max_a;
        }
        else if((acceleration(0) - pre_ax) < -max_a)
        {
            jerk_x = -max_a / 10;
            temp_ax = pre_ax;
            pre_ax -= max_a;
        }
        else
        {
            jerk_x = (acceleration(0) - pre_ax) / 10;
            temp_ax = pre_ax;
            pre_ax = acceleration(0);
        }

        if((acceleration(1) - pre_ay) > max_a)
        {
            jerk_y = max_a / 10;
            temp_ay = pre_ay;
            pre_ay += max_a;
        }
        else if((acceleration(1) - pre_ay) < -max_a)
        {
            jerk_y = -max_a / 10;
            temp_ay = pre_ay;
            pre_ay -= max_a;
        }
        else
        {
            jerk_y = (acceleration(1) - pre_ay) / 10;
            temp_ay = pre_ay;
            pre_ay = acceleration(1);
        }
    }
    else
    {
        pre_relative_p << tar(0), tar(1), 0;
        start_t = ros::Time::now().toSec();
        pre_t = start_t;
    }
    is_init = true;
}

void traj_trigger_callback(const geometry_msgs::PoseStamped::ConstPtr& trigger_msg)
{
    if(is_init)
    {
        ROS_INFO("[#INFO] get traj trigger info");
        traj_id_send += 1;
        is_triggered = true;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "proposed");
    ros::NodeHandle nh("~");
    body_pose_sub = nh.subscribe("body_pose", 100, body_pose_callback);
    trigger_sub = nh.subscribe( "trigger", 10, traj_trigger_callback);
    marker_pose_sub = nh.subscribe("marker_pose", 100, marker_pose_callback);

    position_cmd_pub= nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);

    ros::spin();
    return 0;
}
