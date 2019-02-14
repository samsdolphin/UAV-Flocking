#include <iostream>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sys/time.h>
#include <string>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_generator_waypoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#define uav_max_vx 1.0
#define uav_max_vy 1.0

#define uav_height 2.0

using namespace Eigen;
using namespace std;

ros::Publisher traj_pub;

ros::Publisher position_cmd_pub;

ros::Publisher uav_odom_pub;

ros::Subscriber uav_pos_sub;

ros::Subscriber uav_frontier_sub;

ros::Subscriber uav_waypoints_sub;

ros::Subscriber trigger_sub;

visualization_msgs::Marker selected_marker;

double uav_frontier_point[2]= {0.0,0.0};
VectorXd uav_pos= VectorXd::Zero(4);
MatrixXd uav_coef = MatrixXd::Zero(1,18);
VectorXd uav_t= VectorXd::Zero(1);
Vector2d uav_acc;

int traj_id_send= 0;
double traj_start_time= -1.0;
bool is_first_odom= 0;
bool is_init= 0;
bool is_traj= 0;
bool fail_safe = 0;

ofstream uav_position;

void traj_viz(){
	double des_x = 0, des_y = 0;
    geometry_msgs::Point parent, child;
    parent.z = child.z = 0;

    Eigen::MatrixXd coef_x,coef_y;

    selected_marker.header.stamp = ros::Time::now();
    selected_marker.header.frame_id = "world";
    selected_marker.action = visualization_msgs::Marker::ADD;
    selected_marker.pose.orientation.w = 1.0;
    selected_marker.id = 0;
    selected_marker.type = visualization_msgs::Marker::LINE_LIST;
    selected_marker.scale.x = 0.1;
    selected_marker.color.b = selected_marker.color.a = 1.0;

    bool first=true;
    for(double dT = 0;dT< uav_t(uav_t.rows()-1);dT+=0.1) {
        if(first){
            parent.x=uav_coef(0,0);
            parent.y=uav_coef(0,6);
         	parent.z = 2.0;
            selected_marker.points.push_back(parent);
            first=false;
        }
        for (int i = 0; i < uav_t.size(); i++) {
            if (dT < uav_t(i)) {
                double tt = i > 0 ? dT - uav_t(i - 1) : dT;
                Eigen::Matrix<double, 1, 6> t_p;
                t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                Eigen::Matrix<double, 1, 6> t_v;
                t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);

                Eigen::VectorXd coef_x;
                Eigen::VectorXd coef_y;
                coef_x = (uav_coef.block(i, 0, 1, 6)).transpose();
                coef_y = (uav_coef.block(i, 6, 1, 6)).transpose();

                des_x = t_p * coef_x;
                des_y = t_p * coef_y;
                child.x = des_x;
                child.y = des_y;
                parent.x=des_x;
                parent.y=des_y;
                
                child.z = parent.z = 2.0;
                
                selected_marker.points.push_back(child);
                selected_marker.points.push_back(parent);
                break;
            }
        }
    }

	if (!selected_marker.points.empty()) selected_marker.points.pop_back();


	traj_pub.publish(selected_marker);
    selected_marker.points.clear();
}

void uav_frontier_call_back(const geometry_msgs::PointStamped& msg){
	if(!is_traj) return;
	if(abs(msg.point.x-uav_frontier_point[0])+abs(msg.point.y-uav_frontier_point[1])<0.5 && fail_safe) return;

	uav_frontier_point[0]= msg.point.x;
	uav_frontier_point[1]= msg.point.y;
    fail_safe = (msg.point.z<0);

    if(fail_safe){
    	TrajectoryGeneratorWaypoint fail;
        Eigen::MatrixXd Path = Eigen::MatrixXd::Zero(2,3);
        Eigen::MatrixXd Vel = Eigen::MatrixXd::Zero(2,3);
        Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(2,3);
        Eigen::VectorXd Time = Eigen::VectorXd::Zero(1);
                        
        Path(0,0) = uav_pos(0);
        Path(0,1) = uav_pos(2);
        Path(1,0)= msg.point.x;
        Path(1,1)= msg.point.y;
                        
        Vel(0,0) = uav_pos(1);
        Vel(0,1) = uav_pos(3);
        Vel(1,0) = 0;//uav_pos(1);
        Vel(1,1) = 0;//uav_pos(3);

        Acc(0,0) = uav_acc(0);
        Acc(0,1) = uav_acc(1);
                        
        Time(0) = ((Path.row(0)-Path.row(1)).norm())/sqrt(uav_max_vx*uav_max_vx+uav_max_vy*uav_max_vy)*3;//(Vel.row(0).norm());//
        uav_t.resize(1);

        uav_coef = fail.PolyQPGeneration(Path,Vel,Acc,Time);
        traj_id_send++;
        is_first_odom = 1;
        uav_t(0)= Time(0);
        ROS_ERROR("UAV motion primitive fial coef generated.");
        traj_viz();
    }

}

void uav_waypoints_call_back(const geometry_msgs::PoseArray& msg){
    if(!is_traj) return;
    std::vector<geometry_msgs::Pose> way_pose = msg.poses;
    TrajectoryGeneratorWaypoint T;
    Eigen::MatrixXd Path;
    Eigen::MatrixXd Vel = Eigen::MatrixXd::Zero(2,3);
    Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(2,3);
    Eigen::VectorXd Time;

    Path.resize(way_pose.size()-1,3);
    Time.resize(way_pose.size()-2);
    uav_t.resize(way_pose.size()-2);


    Vel(0,0) = uav_pos(1);
    Vel(0,1) = uav_pos(3);
    Vel(1,0) = way_pose[0].position.x;
    Vel(1,1) = way_pose[0].position.y;

    double tree_time_step = way_pose[0].position.z;
    
    Acc(0,0) = uav_acc(0);
    Acc(0,1) = uav_acc(1);

    Path(0,0) = uav_pos(0);
    Path(0,1) = uav_pos(2);

    way_pose.pop_back();

    for(int i=0; way_pose.size()>1; i++){
        Path(i+1,0) = way_pose.back().position.x;
        Path(i+1,1) = way_pose.back().position.y;
        Time(i)= tree_time_step;
        uav_t(i)= i? uav_t(i-1)+tree_time_step:tree_time_step;
        way_pose.pop_back();
    }

    uav_coef = T.PolyQPGeneration(Path,Vel,Acc,Time);
    fail_safe = 0;
    is_first_odom = 1;
    traj_id_send++;
    ROS_WARN("UAV motion primitive coef generated.");
    traj_viz();

}

void uav_pos_call_back(const nav_msgs::Odometry& msg){
	uav_pos(0)= msg.pose.pose.position.x;
	uav_pos(2)= msg.pose.pose.position.y;
	uav_pos(1)= msg.twist.twist.linear.x;
	uav_pos(3)= msg.twist.twist.linear.y;

    uav_acc(0)= msg.twist.twist.angular.x;
    uav_acc(1)= msg.twist.twist.angular.y;

	if(is_first_odom) {
		traj_start_time= msg.header.stamp.toSec();
		is_first_odom= false;
	}
	if(traj_start_time< 0 && is_init) return;
	if(is_init){
		if(is_traj) {
            double dT = msg.header.stamp.toSec()-traj_start_time;
            double des_x = 0;
            double des_y = 0;
            double des_vx = 0;
            double des_vy = 0;
		    double des_ax = 0;
		    double des_ay = 0;
            bool traj_ok=false;

            for (int i = 0; i < uav_t.size(); i++) {
                if (dT < uav_t(i)) {
                	double tt = i ? dT - uav_t(i - 1) : dT;
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
                    
            if(traj_ok){

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
                des_pos.pose.pose.position.z = msg.pose.pose.position.z;

                des_pos.twist.twist.linear.x = des_vx;
                des_pos.twist.twist.linear.y = des_vy;
                des_pos.twist.twist.linear.z = 0;

                des_pos.twist.twist.angular.x = 0;
                des_pos.twist.twist.angular.y = 0;
                des_pos.twist.twist.angular.z = 0;
                uav_odom_pub.publish(des_pos);

			}
            else{
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
            uav_position<<msg.pose.pose.position.x<<"\t"<<msg.pose.pose.position.y<<"\n";
        }
    }
    else{
        is_init = true;
    }
}

void trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg ){
    if ( is_init ){
        std::cout << "[#INFO] get traj trigger info." << std::endl;
        traj_id_send= traj_id_send + 1;//trigger_msg->header.seq + 1;
        is_traj    = true;
    }
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "harmonic_uav_controller");
	ros::NodeHandle nh;
	uav_frontier_sub = nh.subscribe("/harmonic_duet_node/uav_goal_point", 10,uav_frontier_call_back);
    uav_waypoints_sub = nh.subscribe("/harmonic_duet_node/uav_waypoints", 10,uav_waypoints_call_back);
	uav_pos_sub = nh.subscribe("/vins_estimator/imu_propagate", 10,uav_pos_call_back);
	trigger_sub = nh.subscribe( "/traj_start_trigger", 100, trigger_callback);
	position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
	uav_odom_pub = nh.advertise<nav_msgs::Odometry>("/uav_target",1);
	traj_pub = nh.advertise<visualization_msgs::Marker>("traj_viz", 1);
    uav_position.open("/home/gf/Documents/explore_progress_log/uav_position.txt");
	
	ros::spin();
	return 0;
}