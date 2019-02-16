#include <iostream>
#include <trajectory_generator_waypoint.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;
using namespace std;

ros::Publisher position_cmd_pub;
ros::Publisher uav_odom_pub;
ros::Publisher traj_pub;
ros::Subscriber body_pose_sub;
ros::Subscriber trigger_sub;
visualization_msgs::Marker selected_marker_;

#define uav_height 1.3
#define num_p 3

#define pt1x 0.5
#define pt1y 0.5

#define pt2x 1
#define pt2y -0.5

#define pt3x 1.5
#define pt3y 0.5

int cnt = -1;
int point_pass = 0;
double plan_time_start = 0;
int traj_id_send = 0;
double traj_start_time = -1.0;
bool is_first_odom = 0;
bool is_init = 0;
bool is_traj = 0;

Matrix<double, 3, 2> pts;
MatrixXd uav_coef;
VectorXd uav_t;

void traj_viz()
{
    double des_x = 0, des_y = 0;
    geometry_msgs::Point parent, child;
    parent.z = child.z = 0;
    Eigen::MatrixXd coef = uav_coef;
    Eigen::VectorXd T = uav_t;
    Eigen::MatrixXd coef_x,coef_y;

    selected_marker_.header.stamp = ros::Time::now();

    bool first = true;

    for(double dT=ros::Time::now().toSec(); dT<T(T.rows()-1); dT+=0.1) 
    {
        if(first)
        {
            parent.x = coef(0,0);
            parent.y = coef(0,6);
            parent.z = 2.0;
            selected_marker_.points.push_back(parent);
            first=false;
        }
        for (int i=0; i<T.size(); i++)
        {
            if (dT<T(i))
            {
                double tt = i > 0 ? dT - T(i - 1) : dT - plan_time_start;

                //std::cout<<"T(i-1): "<<T(i-1)<<'\n';

                Eigen::Matrix<double, 1, 6> t_p;
                t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                Eigen::Matrix<double, 1, 6> t_v;
                t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);

                Eigen::VectorXd coef_x;
                Eigen::VectorXd coef_y;
                coef_x = (coef.block(i, 0, 1, 6)).transpose();
                coef_y = (coef.block(i, 6, 1, 6)).transpose();

                des_x = t_p * coef_x;
                des_y = t_p * coef_y;
                child.x = des_x;
                child.y = des_y;
                parent.x=des_x;
                parent.y=des_y;

                child.z = parent.z = 2.0;

                selected_marker_.points.push_back(child);
                selected_marker_.points.push_back(parent);
                break;
            }
        }
    }

    if (!selected_marker_.points.empty())
        selected_marker_.points.pop_back();

    selected_marker_.header.frame_id = "world";
    selected_marker_.action = visualization_msgs::Marker::ADD;
    selected_marker_.pose.orientation.w = 1.0;
    selected_marker_.id = 0;
    selected_marker_.type = visualization_msgs::Marker::LINE_LIST;
    selected_marker_.scale.x = 0.02;
    selected_marker_.color.b = selected_marker_.color.a = 1.0;
    traj_pub.publish(selected_marker_);
    selected_marker_.points.clear();
}

void pub_cmd(const nav_msgs::Odometry& msg)
{
    if(is_first_odom)
    {
        traj_start_time= msg.header.stamp.toSec();
        is_first_odom= false;
    }
    if(traj_start_time< 0 && is_init)
        return;
    if(is_init)
    {
        if(is_traj)
        {
            double dT = msg.header.stamp.toSec();
            double des_x = 0;
            double des_y = 0;
            double des_vx = 0;
            double des_vy = 0;
            double des_ax = 0;
            double des_ay = 0;
            bool traj_ok=false;

            //cout<<dT<<endl;

            for(int i=0; i<uav_t.size(); i++)
            {
                if(dT < uav_t(i))
                {
                    double tt = i ? dT - uav_t(i - 1) : dT - traj_start_time;
                    Matrix<double, 1, 6> t_p;
                    t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                    Matrix<double, 1, 6> t_v;
                    t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);
                    Matrix<double, 1, 6> t_a;
                    t_a << 0, 0, 2, 6 * tt, 12 * pow(tt,2), 20*pow(tt,3);

                    VectorXd coef_x;
                    VectorXd coef_y;

                    coef_x = (uav_coef.block(i, 0, 1, 6)).transpose();
                    coef_y = (uav_coef.block(i, 6, 1, 6)).transpose();

                    des_x = t_p * coef_x;
                    des_y = t_p * coef_y;
                    des_vx = t_v * coef_x;
                    des_vy = t_v * coef_y;
                    des_ax = t_a * coef_x;
                    des_ay = t_a * coef_y;
                    traj_ok= true;

                    break;
                }
            }

            quadrotor_msgs::PositionCommand position_cmd;
            position_cmd.header.stamp    = msg.header.stamp;
            position_cmd.header.frame_id = "world";

            if(traj_ok)
            {
                position_cmd.position.x      = des_x;
                position_cmd.position.y      = des_y;
                position_cmd.position.z      = uav_height;
                position_cmd.velocity.x      = des_vx;
                position_cmd.velocity.y      = des_vy;
                position_cmd.velocity.z      = 0;
                position_cmd.acceleration.x  = des_ax;
                position_cmd.acceleration.y  = des_ay;
                position_cmd.acceleration.z  = 0;
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
                position_cmd.trajectory_id = traj_id_send;

                nav_msgs::Odometry des_pos;
                des_pos.header.stamp    = msg.header.stamp;
                des_pos.header.frame_id = "world";

                des_pos.pose.pose.orientation.w = 1;
                des_pos.pose.pose.orientation.x = 0;
                des_pos.pose.pose.orientation.y = 0;
                des_pos.pose.pose.orientation.z = 0;

                des_pos.pose.pose.position.x = des_x;
                des_pos.pose.pose.position.y = des_y;
                des_pos.pose.pose.position.z = uav_height;

                des_pos.twist.twist.linear.x = des_vx;
                des_pos.twist.twist.linear.y = des_vy;
                des_pos.twist.twist.linear.z = 0;

                des_pos.twist.twist.angular.x = 0;
                des_pos.twist.twist.angular.y = 0;
                des_pos.twist.twist.angular.z = 0;
                uav_odom_pub.publish(des_pos);
            }
            else
            {
                position_cmd.position.x      = msg.pose.pose.position.x;
                position_cmd.position.y      = msg.pose.pose.position.y;
                position_cmd.position.z      = msg.pose.pose.position.z;
                position_cmd.velocity.x      = 0.0;
                position_cmd.velocity.y      = 0.0;
                position_cmd.velocity.z      = 0.0;
                position_cmd.acceleration.x  = 0.0;
                position_cmd.acceleration.y  = 0.0;
                position_cmd.acceleration.z  = 0.0;
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
                position_cmd.trajectory_id   = traj_id_send;
            }

            position_cmd_pub.publish( position_cmd );
        }
    }
    else
    {
        is_init = true;
    }
}

void body_pose_call_back(const nav_msgs::Odometry &msg)
{
    //std::cout<<"cnt:"<<cnt<<"\n";
    is_init = true;

    if(cnt < 0) // Not triggered
        return;

    if(point_pass<3)
    {
        if(cnt > 0)
        {
            cnt++;
            cnt = cnt%1200; // process once every 1000/400 seconds
        }
        else
        {
            ros::Time time0 = ros::Time::now();
            cnt++;
            //if(point_pass>2) point_pass=2;

            plan_time_start = msg.header.stamp.toSec();
            TrajectoryGeneratorWaypoint T;
            Eigen::MatrixXd Path = Eigen::MatrixXd::Zero(4-point_pass, 3);
            Eigen::MatrixXd Vel = Eigen::MatrixXd::Zero(2, 3);
            Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(2, 3);
            Eigen::VectorXd Time = Eigen::VectorXd::Zero(3-point_pass);

            Path(0,0) = msg.pose.pose.position.x;
            Path(0,1) = msg.pose.pose.position.y;
            Path.block(1,0,3-point_pass,2) = pts.block(point_pass,0,3-point_pass,2);

            Vel(0,0) = msg.twist.twist.linear.x;
            Vel(0,1) = msg.twist.twist.linear.y;
            Vel(1,0) = 0;//uav_pos(1);
            Vel(1,1) = 0;//uav_pos(3);

            Acc(0,0) = msg.twist.twist.angular.x;
            Acc(0,1) = msg.twist.twist.angular.y;


            for(int i=0; i<3-point_pass; i++)
                Time(i) = 3; 

            uav_coef = T.PolyQPGeneration(Path,Vel,Acc,Time);
            ros::Time time1_1 = ros::Time::now();
            cout<<"time consume in solver : "<<(time1_1 - time0).toSec()<<endl;

            uav_t.resize(uav_coef.rows());
            uav_t(0) = Time(0)+plan_time_start;
            for(int i=1; i<3-point_pass; i++)
                uav_t(i) = 3+uav_t(i-1);

            ros::Time time1 = ros::Time::now();
            cout<<"time consume : "<<(time1 - time0).toSec()<<endl;

            traj_viz();
            point_pass++;
            is_first_odom = 1;
            traj_id_send++;
            cout<<uav_t<<endl;
        }
    }
    pub_cmd(msg);	
}


void traj_trigger_callback(const geometry_msgs::PoseStamped::ConstPtr& trigger_msg)
{
    cnt = 0;
    if(is_init)
    {
        std::cout << "[#INFO] get traj trigger info." << std::endl;
        traj_id_send = traj_id_send + 1;//trigger_msg->header.seq + 1;
        is_traj = true;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_traj");
    ros::NodeHandle nh;
    body_pose_sub = nh.subscribe("/vins_estimator/imu_propagate", 10, body_pose_call_back);
    trigger_sub = nh.subscribe( "/traj_start_trigger", 100, traj_trigger_callback);

    traj_pub= nh.advertise<visualization_msgs::Marker>("traj_viz", 1);
    position_cmd_pub= nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
    uav_odom_pub = nh.advertise<nav_msgs::Odometry>("/uav_target",1);

    pts<< pt1x,pt1y,pt2x,pt2y,pt3x,pt3y;//,pt4x,pt4y,pt5x,pt5y,pt6x,pt6y,pt7x,pt7y,pt8x,pt8y,pt9x,pt9y,pt10x,pt10y;

    ros::spin();
    return 0;
}
