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
#define max_a 0.25
#define max_v 0.4
#define H 1.0
#define beta 0.25
#define theta 2
#define d0 1
#define d1 8

Vector3d pre_relative_p(0, 0, 0);
double pre_relative_px = 0;
Vector3d tar(0, 0, 0);
Vector3d p(0, 0, uav_height);
Vector3d v(0, 0, 0);

double pre_t = 0.0;
double pre_a = 0.0;
double temp_a = 0.0;
double jerk = 0.0;
double pre_ax = 0.0;
double temp_ax = 0.0;
double jerkx = 0.0;
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
        double acc = 0.0;
        double accx = 0.0;

        if(temp_a + jerk * cnt <= pre_a)
            acc = temp_a + jerk * cnt;
        else
            acc = pre_a;

        if(temp_ax + jerkx * cnt <= pre_ax)
            accx = temp_ax + jerkx * cnt;
        else
            accx = pre_ax;


        velocity(0) = v(0) + accx * t;
        velocity(1) = v(1) + acc * t;
        velocity(2) = 0;

        if(velocity(1)>max_v)
        {
            velocity(1)=max_v;
            acc=0;
        }
        else if(velocity(1)<-max_v)
        {
            velocity(1)=-max_v;
            acc=0;
        }

        if(velocity(0)>max_v)
        {
            velocity(0)=max_v;
            accx=0;
        }
        else if(velocity(0)<-max_v)
        {
            velocity(0)=-max_v;
            accx=0;
        }

        position(0) = p(0) + v(0) * t + 0.5 * accx * pow(t, 2);
        position(1) = p(1) + v(1) * t + 0.5 * acc * pow(t, 2);
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
        cmd.acceleration.y = acc;
        cmd.acceleration.z = 0;
        
        cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id_send;

        position_cmd_pub.publish(cmd);
        loop_t = ros::Time::now().toSec();
        //if(cnt==0)
        //    cout<<p(1)<<endl;
        cnt++;
        cnt = cnt % 30;
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
    double relative_px;
    tar << cmsg->pose.position.x, cmsg->pose.position.y, uav_height;
    //cout<<sqrt(pow(tar(0)-p(0),2)+pow(tar(1)-p(1),2))<<endl;
    //cout<<tar(0)-p(0)<<endl;
    //cout<<tar(1)-p(1)<<endl;
    //cout<<tar(0)<<endl;
    //cout<<tar(1)<<endl;
    //cout<<p(0)<<endl;
    //cout<<p(1)<<endl;

    if(pre_relative_p.norm() > 0)
    {
        delta = ros::Time::now().toSec() - pre_t;
        relative_p << 0, tar(1) - p(1) + sqrt(3.5), 0;
        //cout<<"relative_p: "<<tar(0) - p(0)<<", "<<relative_p(1)<<endl;

        double aij = H / pow((1 + pow(relative_p.norm(), 2)), beta);
        double f = 1 / pow(pow(relative_p.norm(), 2) - d0, theta);
        double g = 1 / pow(pow(relative_p.norm(), 2) - 6, theta);

        relative_v = (relative_p - pre_relative_p)/delta;

        /* psi_scal */
        //cout<<(relative_v+v).dot(v)/v.norm()/(relative_v+v).norm()<<endl;

        double lambda = pow(0.5 * pow(relative_v.norm(), 2), 0.5);

        relative_px = tar(0) - p(0);
        double aijx = H / pow((1 + pow(relative_px, 2)), beta);
        double fx = 1 / pow(pow(relative_px, 2) - d0, theta);
        double gx = 1 / pow(pow(relative_px, 2) - d1, theta);
        double relative_vx = (relative_px - pre_relative_px)/delta;
        double lambdax = pow(0.5 * pow(relative_vx, 2), 0.5);
        pre_relative_px = relative_px;

        pre_relative_p = relative_p;
        pre_t = ros::Time::now().toSec();

        acceleration(0) = aijx * relative_vx - lambdax * fx * relative_px + lambdax * gx * relative_px;
        acceleration(1) = aij * relative_v(1) - lambda * f * relative_p(1) + lambda * g * relative_p(1);
        acceleration(2) = 0;
        //cout<<"des_a: "<<acceleration(0)<<", "<<acceleration(1)<<endl;
 
        if((acceleration(1) - pre_a) > max_a)
        {
            jerk = max_a / 30;
            temp_a = pre_a;
            pre_a += max_a;
        }
        else if((acceleration(1) - pre_a) < -max_a)
        {
            jerk = -max_a / 30;
            temp_a = pre_a;
            pre_a -= max_a;
        }
        else
        {
            jerk = (acceleration(1) - pre_a) / 30;
            temp_a = pre_a;
            pre_a = acceleration(1);
        }

        if((acceleration(0) - pre_ax) > max_a)
        {
            jerkx = max_a / 30;
            temp_ax = pre_ax;
            pre_ax += max_a;
        }
        else if((acceleration(0) - pre_ax) < -max_a)
        {
            jerkx = -max_a / 30;
            temp_ax = pre_ax;
            pre_ax -= max_a;
        }
        else
        {
            jerkx = (acceleration(0) - pre_ax) / 30;
            temp_ax = pre_ax;
            pre_ax = acceleration(0);
        }
    }
    else
    {
        pre_relative_p << 0, tar(1) - p(1) + sqrt(3.5), 0;
        pre_relative_px = tar(0) - p(0);
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
    ros::init(argc, argv, "jiugang");
    ros::NodeHandle nh("~");
    body_pose_sub = nh.subscribe("body_pose", 100, body_pose_callback);
    trigger_sub = nh.subscribe( "trigger", 10, traj_trigger_callback);
    marker_pose_sub = nh.subscribe("marker_pose", 100, marker_pose_callback);

    position_cmd_pub= nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);

    ros::spin();
    return 0;
}
