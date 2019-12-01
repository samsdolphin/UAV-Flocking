#include <iostream>
#include <utility> // std::pair
#include <cmath>
#include <vector>
#include <list>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <trajectory_generator_waypoint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>

using namespace Eigen;
using namespace std;
using namespace message_filters;

#define _NUM_P 9
#define _LAMBDA_T 0.1
#define _LAMBDA_F 0.01
#define _MAX_V 0.8
#define _MAX_A 1.0

ros::Publisher tar_traj_pub;
ros::Publisher tar_pt_pub;
ros::Publisher plan_traj_pub;
ros::Publisher position_cmd_pub;

ros::Subscriber body_pose_sub;
ros::Subscriber trigger_sub;
ros::Subscriber marker_pose_sub;

visualization_msgs::Marker target_traj;
visualization_msgs::Marker target_pt;
visualization_msgs::Marker uav_plan_traj;

int p_cnt = 0;
int imu_cnt = 1;
int traj_id_send = 0;
int uav_state = 0;

bool is_init = false;
bool is_triggered = false;
bool target_hover = false;

double t_start = 0.0;
double t_plan = 0.0;
double t_lm = 0.0;
double uav_t = 0.0;
double uav_height = 1.15;

Vector3d body_pose, des_body_hover;
Vector3d marker_hover(0, 0, 0);
Vector3d marker_pose(0, 0, 0);
Vector3d temp_body_pose(0, 0, 0);
Matrix3d body_orien;
MatrixXd uav_coef;
VectorXd coef, _coef, _P;
Matrix3d imu_R_cam;
list<pair<double, Vector3d>> tar_obs;
list<Vector3d> marker_obs;

void target_viz()
{
	geometry_msgs::Point target;
	target_pt.header.stamp = ros::Time::now();
	for (int i=0; i<_NUM_P; i++)
	{
		target.x = _P(3*i);
		target.y = _P(3*i+1);
		target.z = _P(3*i+2);
		target_pt.points.push_back(target);
		//cout<<"pt: "<<target.x<<" "<<target.y<<" "<<target.z<<" "<<endl;
	}
	target_pt.header.frame_id = "world";
    target_pt.action = visualization_msgs::Marker::ADD;
    target_pt.pose.orientation.w = 1.0;
    target_pt.id = 0;
    target_pt.type = visualization_msgs::Marker::POINTS;
    target_pt.scale.x = 0.01;
    target_pt.scale.y = 0.01;
    target_pt.color.g = target_pt.color.a = 1.0;
    tar_pt_pub.publish(target_pt);
    target_pt.points.clear();
}

void traj_viz()
{
    geometry_msgs::Point parent;
    target_traj.header.stamp = ros::Time::now();
    VectorXd ctemp;
	ctemp = VectorXd::Zero(6);
	//ctemp(0) = -1.5;

    for (double dT=0; dT<t_lm; dT+=t_lm/20)
    {
        Matrix<double, 1, 6> t_p;
        t_p << 1, dT, pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);

        parent.x = t_p*(coef.segment<6>(0)+ctemp);
        parent.y = t_p*coef.segment<6>(6);
        parent.z = t_p*coef.segment<6>(12);
        target_traj.points.push_back(parent);
    }

    target_traj.header.frame_id = "world";
    target_traj.action = visualization_msgs::Marker::ADD;
    target_traj.pose.orientation.w = 1.0;
    target_traj.id = 1;
    target_traj.type = visualization_msgs::Marker::LINE_STRIP;
    target_traj.scale.x = 0.01;
    target_traj.color.b = target_traj.color.a = 1.0;
    tar_traj_pub.publish(target_traj);
    target_traj.points.clear();
}

void f_viz(const MatrixXd &_cef)
{
    geometry_msgs::Point parent;
    uav_plan_traj.header.stamp = ros::Time::now();

    for (double dT=0; dT<t_lm/2; dT+=t_lm/20)
    {
        Matrix<double, 1, 6> t_p;
        t_p << 1, dT, pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);
    	VectorXd xcoef = _cef.block<1, 6>(0, 0);
    	VectorXd ycoef = _cef.block<1, 6>(0, 6);
    	VectorXd zcoef = _cef.block<1, 6>(0, 12);

        parent.x = t_p*xcoef;
        parent.y = t_p*ycoef;
        parent.z = t_p*zcoef;
        uav_plan_traj.points.push_back(parent);
    }

    uav_plan_traj.header.frame_id = "world";
    uav_plan_traj.action = visualization_msgs::Marker::ADD;
    uav_plan_traj.pose.orientation.w = 1.0;
    uav_plan_traj.id = 2;
    uav_plan_traj.type = visualization_msgs::Marker::LINE_STRIP;
    uav_plan_traj.scale.x = 0.01;
    uav_plan_traj.color.r = uav_plan_traj.color.a = 1.0;
    plan_traj_pub.publish(uav_plan_traj);
    uav_plan_traj.points.clear();
}

void f_viz(const VectorXd &_coef)
{
    geometry_msgs::Point parent;
    uav_plan_traj.header.stamp = ros::Time::now();

    for (double dT=0; dT<t_lm/2; dT+=t_lm/20)
    {
        Matrix<double, 1, 6> t_p;
        t_p << 1, dT, pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);

        parent.x = t_p*_coef.segment<6>(0);
        parent.y = t_p*_coef.segment<6>(6);
        parent.z = t_p*_coef.segment<6>(12);
        uav_plan_traj.points.push_back(parent);
    }

    uav_plan_traj.header.frame_id = "world";
    uav_plan_traj.action = visualization_msgs::Marker::ADD;
    uav_plan_traj.pose.orientation.w = 1.0;
    uav_plan_traj.id = 2;
    uav_plan_traj.type = visualization_msgs::Marker::LINE_STRIP;
    uav_plan_traj.scale.x = 0.01;
    uav_plan_traj.color.r = uav_plan_traj.color.a = 1.0;
    plan_traj_pub.publish(uav_plan_traj);
    uav_plan_traj.points.clear();
}

void estimate_T(const list<pair<double, Vector3d>> &tar_ob)
{
	_P = VectorXd::Zero(3*_NUM_P);
	MatrixXd _T = MatrixXd::Zero(3*_NUM_P, 18);
	MatrixXd _t = MatrixXd::Zero(1, 6);
	MatrixXd __T = MatrixXd::Zero(18, 18);
	MatrixXd __t = MatrixXd::Zero(4, 4);
	MatrixXd _para = MatrixXd::Zero(4, 4);
	t_start = tar_ob.front().first;
	t_lm = 2*(tar_ob.back().first-tar_ob.front().first);
	//cout<<"image time length :"<<tar_ob.back().first-tar_ob.front().first<<endl;

	__t << 4, 6, 8, 10,
		   6, 12, 18, 24,
		   8, 18, 28.8, 40,
		   10, 24, 40, 57.1429;

	_para << t_lm, pow(t_lm, 2), pow(t_lm, 3), pow(t_lm, 4),
			 pow(t_lm, 2), pow(t_lm, 3), pow(t_lm, 4), pow(t_lm, 5),
			 pow(t_lm, 3), pow(t_lm, 4), pow(t_lm, 5), pow(t_lm, 6),
			 pow(t_lm, 4), pow(t_lm, 5), pow(t_lm, 6), pow(t_lm, 7);

	for (int i=0; i<3; i++)
		__T.block<4, 4>(6*i+2, 6*i+2) = __t.cwiseProduct(_para);

	for (list<pair<double, Vector3d>>::const_iterator it=tar_ob.begin(), end=tar_ob.end() ; it!=end; ++it)
	{
		double dT = it->first-t_start;
		int i = distance(tar_ob.begin(), it);

		_t << 1, pow(dT, 1), pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);
		_T.block<1, 6>(3*i, 0) = _t;
		_T.block<1, 6>(3*i+1, 6) = _t;
		_T.block<1, 6>(3*i+2, 12) = _t;
		_P.segment<3>(3*i) = it->second;
	}

	int nx = 18;
	int my = 0;
	int mz = 0;
	int nnzA = 0;
	int nnzC = 0;
	int nnzQ = 0;
	int irowA[my], jcolA[my], irowC[mz], jcolC[mz];
	double dA[my], bA[my], dC[mz];

	double c[nx];
	VectorXd vtemp;
	vtemp = -_T.transpose()*_P;
	for (int i=0; i<nx; i++)
		c[i] = vtemp(i);
	MatrixXd _Q = MatrixXd::Zero(18, 18);
	_Q = _T.transpose()*_T+_LAMBDA_T*__T;

	vector<pair<pair<int, int>, double> > tmp;
	for (int i=0; i<_Q.rows(); i++)
		for (int j=0; j<_Q.cols(); j++)
			if (j<=i && _Q(i, j)>1e-2)
				nnzQ++;

	int irowQ[nnzQ], jcolQ[nnzQ], i_q = 0;
	double dQ[nnzQ];

	tmp.resize(nnzQ);
	for (int i=0; i<_Q.rows(); i++)
		for (int j=0; j<_Q.cols(); j++)
			if (j<=i && _Q(i, j)>1e-2)
				tmp[i_q++] = make_pair(make_pair(i, j), _Q(i, j));

	sort(tmp.begin(), tmp.end());

	for (unsigned int i=0; i<tmp.size(); ++i)
	{
		irowQ[i] = tmp[i].first.first;
		jcolQ[i] = tmp[i].first.second;
		dQ[i] = tmp[i].second;
	}

	double xupp[nx];
	char ixupp[nx];
	for (int i = 0 ; i < nx; i++)
		xupp[i] = 0.0;
	memset(ixupp, 0, sizeof(ixupp));

	double xlow[nx];    
	char ixlow[nx];
	for (int i = 0; i < nx; i++)
		xlow[i] = 0.0;
	memset(ixlow, 0, sizeof(ixlow));

	QpGenSparseMa27 *qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
	QpGenData *prob = (QpGenData*)qp->copyDataFromSparseTriple(c, irowQ, nnzQ, jcolQ, dQ,
															   xlow, ixlow, xupp, ixupp,
															   irowA, nnzA, jcolA, dA, bA,
															   irowC, nnzC, jcolC, dC,
															   xlow, ixlow, xupp, ixupp);
	QpGenVars *vars = (QpGenVars*) qp->makeVariables(prob);
	QpGenResiduals *resid = (QpGenResiduals*) qp->makeResiduals(prob);
	GondzioSolver *s = new GondzioSolver(qp, prob);
	//s->monitorSelf();
	int status = s->solve(prob, vars, resid);
	delete s;
	delete qp;
	coef = VectorXd::Zero(nx);
	double d[nx];
	vars->x->copyIntoArray(d);
	for (int i=0; i<nx; i++)
		coef(i) = d[i];
	target_viz();
	traj_viz();
	//cout<<"coef:\n"<<coef.head<6>()<<endl;
}

void estimate_F(const nav_msgs::Odometry &bmsg)
{
	MatrixXd _T = MatrixXd::Zero(18, 18);
	MatrixXd _t = MatrixXd::Zero(3, 3);
	MatrixXd _para = MatrixXd::Zero(3, 3);
	MatrixXd __T = MatrixXd::Zero(18, 18);
	MatrixXd __t = MatrixXd::Zero(6, 6);
	MatrixXd __para = MatrixXd::Zero(6, 6);
	MatrixXd _TS = MatrixXd::Zero(18, 18);
	MatrixXd _ts = MatrixXd::Zero(6, 6);
	MatrixXd _VA = MatrixXd::Zero(2*_NUM_P*3, 18);
	MatrixXd _v = MatrixXd::Zero(1, 5);
	MatrixXd _vp = MatrixXd::Zero(1, 5);
	MatrixXd _a = MatrixXd::Zero(1, 4);
	MatrixXd _ap = MatrixXd::Zero(1, 4);

	double t = 0.5*t_lm;
	double dT = t;

	__t << 1, 0.5, 0.3333, 0.25, 0.2, 0.1667,
		   0.5, 0.3333, 0.25, 0.2, 0.1667, 0.1429,
		   0.3333, 0.25, 0.2, 0.1667, 0.1429, 0.125,
		   0.25, 0.2, 0.1667, 0.1429, 0.125, 0.1111,
		   0.2, 0.1667, 0.1429, 0.125, 0.1111, 0.1,
		   0.1667, 0.1429, 0.125, 0.1111, 0.1, 0.0909;

	__para << t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6),
			  pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7),
			  pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7), pow(t, 8),
			  pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7), pow(t, 8), pow(t, 9),
			  pow(t, 5), pow(t, 6), pow(t, 7), pow(t, 8), pow(t, 9), pow(t, 10),
			  pow(t, 6), pow(t, 7), pow(t, 8), pow(t, 9), pow(t, 10), pow(t, 11);

	_t << 36, 72, 120,
		  72, 192, 360,
		  120, 360, 720;

	_para << t, pow(t, 2), pow(t, 3),
			 pow(t, 2), pow(t, 3), pow(t, 4),
			 pow(t, 3), pow(t, 4), pow(t, 5);

	for (int i=0; i<6; i++)
	{
		_ts(0, i) = 1.0/(i+1)*pow(t, i+1);
		_ts(1, i) = 1.0/(i+2)*pow(t, i+2)+1.0/(i+1)*pow(t, i+1)*dT;
		_ts(2, i) = 1.0/(i+3)*pow(t, i+3)+1.0/(i+2)*pow(t, i+2)*2*dT+1.0/(i+1)*pow(t, i+1)*pow(dT, 2);
		_ts(3, i) = 1.0/(i+4)*pow(t, i+4)+1.0/(i+3)*pow(t, i+3)*3*dT+1.0/(i+2)*pow(t, i+2)*3*pow(dT, 2)+1.0/(i+1)*pow(t, i+1)*pow(dT, 3);
		_ts(4, i) = 1.0/(i+5)*pow(t, i+5)+1.0/(i+4)*pow(t, i+4)*4*dT+1.0/(i+3)*pow(t, i+3)*6*pow(dT, 2)+1.0/(i+2)*pow(t, i+2)*4*pow(dT, 3)
					+1.0/(i+1)*pow(t, i+1)*pow(dT, 4);
		_ts(5, i) = 1.0/(i+6)*pow(t, i+6)+1.0/(i+5)*pow(t, i+5)*5*dT+1.0/(i+4)*pow(t, i+4)*10*pow(dT, 2)+1.0/(i+3)*pow(t, i+3)*10*pow(dT, 3)
					+1.0/(i+2)*pow(t, i+2)*5*pow(dT, 4)+1.0/(i+1)*pow(t, i+1)*pow(dT, 5);
	}

	/* SPEED, ACCELERATION BOUND */
	for (int i=0; i<_NUM_P; i++)
	{
		double dT = t/_NUM_P*i;
		//cout<<"dT: "<<dT<<endl;
		_v << 1, 2, 3, 4, 5;
		_vp << 1, dT, pow(dT, 2), pow(dT, 3), pow(dT, 4);
		_a << 2, 6, 12, 20;
		_ap << 1, dT, pow(dT, 2), pow(dT, 3);
		_VA.block<1, 5>(i*3, 1) = _v.cwiseProduct(_vp);
		_VA.block<1, 5>(i*3+1, 7) = _v.cwiseProduct(_vp);
		_VA.block<1, 5>(i*3+2, 13) = _v.cwiseProduct(_vp);
		_VA.block<1, 4>((i+_NUM_P)*3, 2) = _a.cwiseProduct(_ap);
		_VA.block<1, 4>((i+_NUM_P)*3+1, 8) = _a.cwiseProduct(_ap);
		_VA.block<1, 4>((i+_NUM_P)*3+2, 14) = _a.cwiseProduct(_ap);
	}

	for (int i=0; i<3; i++)
	{
		__T.block<6, 6>(6*i, 6*i) = __t.cwiseProduct(__para);
		_T.block<3, 3>(6*i+3, 6*i+3) = _t.cwiseProduct(_para);
		_TS.block<6, 6>(6*i, 6*i) = _ts;
	}

	int nx = 18;
	int my = 9;
	int mz = 3*_NUM_P*2;
	int nnzA = 9;
	int nnzC = 0; //249
	int nnzQ = 0; //63

	int irowA[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	int jcolA[] = {0, 6, 12, 1, 7, 13, 2, 9, 14};
	double dA[] = {1, 1, 1, 1, 1, 1, 2, 2, 2};
	double bA[] = {bmsg.pose.pose.position.x, bmsg.pose.pose.position.y, bmsg.pose.pose.position.z,
				   bmsg.twist.twist.linear.x, bmsg.twist.twist.linear.y, bmsg.twist.twist.linear.z,
				   bmsg.twist.twist.angular.x, bmsg.twist.twist.angular.y, bmsg.twist.twist.angular.z};

	for (int i=0; i<_VA.rows(); i++)
		for (int j=0; j<_VA.cols(); j++)
			if (_VA(i, j)>1e-4)
				nnzC++;

	int irowC[nnzC], jcolC[nnzC], i_c = 0;
	double dC[nnzC], cupp[3*_NUM_P*2], clow[3*_NUM_P*2];
	char icupp[3*_NUM_P*2], iclow[3*_NUM_P*2];

	for (int i=0; i<_VA.rows(); i++)
	{
		icupp[i] = 1;
		iclow[i] = 1;
		if (i<3*_NUM_P)
		{
			cupp[i] = _MAX_V;
			clow[i] = -_MAX_V;
		}
		else
		{
			cupp[i] = _MAX_A;
			clow[i] = -_MAX_A;
		}
		for (int j=0; j<_VA.cols(); j++)
			if (_VA(i, j)>1e-4)
			{
				irowC[i_c] = i;
				jcolC[i_c] = j;
				dC[i_c] = _VA(i, j);
				i_c++;
			}
	}

	double c[nx];
	VectorXd vtemp, ctemp;
	ctemp = VectorXd::Zero(nx);
	ctemp(0) = -1.5;
	vtemp = -_TS.transpose()*(coef+ctemp);
	for (int i=0; i<nx; i++)
		c[i] = vtemp(i);

	MatrixXd _Q = MatrixXd::Zero(18, 18);
	_Q = __T+_LAMBDA_F*_T;

	vector<pair<pair<int, int>, double> > tmp;
	for (int i=0; i<_Q.rows(); i++)
		for (int j=0; j<_Q.cols(); j++)
			if (j<=i && _Q(i, j)>1e-4)
				nnzQ++;

	int irowQ[nnzQ], jcolQ[nnzQ], i_q = 0;
	double dQ[nnzQ];

	tmp.resize(nnzQ);
	for (int i=0; i<_Q.rows(); i++)
		for (int j=0; j<_Q.cols(); j++)
			if (j<=i && _Q(i, j)>1e-4)
				tmp[i_q++] = make_pair(make_pair(i, j), _Q(i, j));

	sort(tmp.begin(), tmp.end());

	for (unsigned int i=0; i<tmp.size(); ++i)
	{
		irowQ[i] = tmp[i].first.first;
		jcolQ[i] = tmp[i].first.second;
		dQ[i] = tmp[i].second;
	}

	double xupp[nx];
	char ixupp[nx];
	for (int i = 0 ; i < nx; i++)
		xupp[i] = 0.0;
	memset(ixupp, 0, sizeof(ixupp));

	double xlow[nx];    
	char ixlow[nx];
	for (int i = 0; i < nx; i++)
		xlow[i] = 0.0;
	memset(ixlow, 0, sizeof(ixlow));

	QpGenSparseMa27 *qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
	QpGenData *prob = (QpGenData*)qp->copyDataFromSparseTriple(c, irowQ, nnzQ, jcolQ, dQ,
															   xlow, ixlow, xupp, ixupp,
															   irowA, nnzA, jcolA, dA, bA,
															   irowC, nnzC, jcolC, dC,
															   clow, iclow, cupp, icupp);
	QpGenVars *vars = (QpGenVars*) qp->makeVariables(prob);
	QpGenResiduals *resid = (QpGenResiduals*) qp->makeResiduals(prob);
	GondzioSolver *s = new GondzioSolver(qp, prob);
	//s->monitorSelf();
	int status = s->solve(prob, vars, resid);
	delete s;
	delete qp;
	_coef = VectorXd::Zero(nx);
	double d[nx];
	vars->x->copyIntoArray(d);
	for (int i=0; i<nx; i++)
		_coef(i) = d[i];
	f_viz(_coef);
}

void body_pose_callback(const nav_msgs::Odometry &msg)
{
	Vector3d cur_body_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

	if (imu_cnt>0)
	{
		temp_body_pose += cur_body_pose;
		imu_cnt++;
		imu_cnt = imu_cnt%40;
	}
	else
	{
		imu_cnt++;
		body_pose = temp_body_pose/39; // update body pose at 10Hz
		temp_body_pose << 0, 0, 0;
	}

	if (is_triggered)
	{
		quadrotor_msgs::PositionCommand position_cmd;
	    position_cmd.header.stamp = msg.header.stamp;
	    position_cmd.header.frame_id = "world";

		switch (uav_state)
		{
			case 0: // HOVER_STATE
			if (target_hover)
			{
				if ((des_body_hover.head<2>()-body_pose.head<2>()).norm()>0.1)
				{
					//cout<<"des hover: "<<des_body_hover.transpose()<<endl;
					//cout<<"body_pose: "<<body_pose.transpose()<<endl;
					cout<<"GO for LINE"<<endl;
					TrajectoryGeneratorWaypoint T;

					MatrixXd Path = MatrixXd::Zero(2, 3);
		            MatrixXd Vel = MatrixXd::Zero(2, 3);
		            MatrixXd Acc = MatrixXd::Zero(2, 3);
		            VectorXd Time = VectorXd::Zero(1);

		            Path(0, 0) = msg.pose.pose.position.x;
		            Path(0, 1) = msg.pose.pose.position.y;
		            Path(1, 0) = des_body_hover(0);
		            Path(1, 1) = des_body_hover(1);

		            Vel(0, 0) = msg.twist.twist.linear.x;
		            Vel(0, 1) = msg.twist.twist.linear.y;
		            Vel(1, 0) = 0; // uav_pos(1);
		            Vel(1, 1) = 0; // uav_pos(3);

		            Acc(0, 0) = msg.twist.twist.angular.x;
		            Acc(0, 1) = msg.twist.twist.angular.y;

		            Time(0) = 2;

		            uav_coef = T.PolyQPGeneration(Path, Vel, Acc, Time);
		            uav_t = msg.header.stamp.toSec();
		            uav_state = 1; // LINE_STATE
		            traj_id_send++;
		            //f_viz(uav_coef);
				}
			}
			else
			{
				cout<<"GO for TRAJ"<<endl;
				estimate_F(msg);
				uav_t = msg.header.stamp.toSec();
	            uav_state = 2; // TRAJ_STATE
	            traj_id_send++;
			}
			//cout<<"CASE 0 HOVER"<<endl;
			position_cmd.position.x = msg.pose.pose.position.x;
	        position_cmd.position.y = msg.pose.pose.position.y;
	        position_cmd.position.z = msg.pose.pose.position.z;
	        position_cmd.velocity.x = 0.0;
	        position_cmd.velocity.y = 0.0;
	        position_cmd.velocity.z = 0.0;
	        position_cmd.acceleration.x = 0.0;
	        position_cmd.acceleration.y = 0.0;
	        position_cmd.acceleration.z = 0.0;
	        position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
	        position_cmd.trajectory_id = traj_id_send;
			break;

			case 1: // LINE_STATE
			if (msg.header.stamp.toSec()-uav_t<=2)
			{
				//cout<<"CASE 1 LINE"<<endl;
				double dT = msg.header.stamp.toSec()-uav_t;
				double des_x = 0;
				double des_y = 0;
				double des_vx = 0;
				double des_vy = 0;
				double des_ax = 0;
				double des_ay = 0;
				Matrix<double, 1, 6> t_p;
				t_p << 1, dT, pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);
				Matrix<double, 1, 6> t_v;
				t_v << 0, 1, 2*dT, 3*pow(dT, 2), 4*pow(dT, 3), 5*pow(dT, 4);
				Matrix<double, 1, 6> t_a;
				t_a << 0, 0, 2, 6*dT, 12*pow(dT,2), 20*pow(dT,3);

				VectorXd coef_x;
				VectorXd coef_y;

				coef_x = (uav_coef.block<1, 6>(0, 0)).transpose();
				coef_y = (uav_coef.block<1, 6>(0, 6)).transpose();

				des_x = t_p*coef_x;
				des_y = t_p*coef_y;
				des_vx = t_v*coef_x;
				des_vy = t_v*coef_y;
				des_ax = t_a*coef_x;
				des_ay = t_a*coef_y;

				position_cmd.position.x = des_x;
				position_cmd.position.y = des_y;
				position_cmd.position.z = uav_height;
				position_cmd.velocity.x = des_vx;
				position_cmd.velocity.y = des_vy;
				position_cmd.velocity.z = 0;
				position_cmd.acceleration.x = des_ax;
				position_cmd.acceleration.y = des_ay;
				position_cmd.acceleration.z = 0;
				position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
				position_cmd.trajectory_id = traj_id_send;
			}
			else
			{
				cout<<"EXIT CASE 1 LINE"<<endl;
				uav_state = 0; // GO BACK TO HOVER_STATE
				position_cmd.position.x = msg.pose.pose.position.x;
		        position_cmd.position.y = msg.pose.pose.position.y;
		        position_cmd.position.z = msg.pose.pose.position.z;
		        position_cmd.velocity.x = 0.0;
		        position_cmd.velocity.y = 0.0;
		        position_cmd.velocity.z = 0.0;
		        position_cmd.acceleration.x = 0.0;
		        position_cmd.acceleration.y = 0.0;
		        position_cmd.acceleration.z = 0.0;
		        position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
		        position_cmd.trajectory_id = traj_id_send;
			}
			break;

			case 2: // TRAJ_STATE
			if (msg.header.stamp.toSec()-uav_t<=0.5*t_lm)
			{
				//cout<<"CASE 1 LINE"<<endl;
				double dT = msg.header.stamp.toSec()-uav_t;
				double des_x = 0;
				double des_y = 0;
				double des_vx = 0;
				double des_vy = 0;
				double des_ax = 0;
				double des_ay = 0;
				Matrix<double, 1, 6> t_p;
				t_p << 1, dT, pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);
				Matrix<double, 1, 6> t_v;
				t_v << 0, 1, 2*dT, 3*pow(dT, 2), 4*pow(dT, 3), 5*pow(dT, 4);
				Matrix<double, 1, 6> t_a;
				t_a << 0, 0, 2, 6*dT, 12*pow(dT,2), 20*pow(dT,3);

				VectorXd coef_x;
				VectorXd coef_y;

				coef_x = _coef.segment<6>(0);
				coef_y = _coef.segment<6>(6);

				des_x = t_p*coef_x;
				des_y = t_p*coef_y;
				des_vx = t_v*coef_x;
				des_vy = t_v*coef_y;
				des_ax = t_a*coef_x;
				des_ay = t_a*coef_y;

				position_cmd.position.x = des_x;
				position_cmd.position.y = des_y;
				position_cmd.position.z = uav_height;
				position_cmd.velocity.x = des_vx;
				position_cmd.velocity.y = des_vy;
				position_cmd.velocity.z = 0;
				position_cmd.acceleration.x = des_ax;
				position_cmd.acceleration.y = des_ay;
				position_cmd.acceleration.z = 0;
				position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
				position_cmd.trajectory_id = traj_id_send;
			}
			else
			{
				cout<<"EXIT CASE 2 TRAJ"<<endl;
				uav_state = 0; // GO BACK TO HOVER_STATE
				position_cmd.position.x = msg.pose.pose.position.x;
		        position_cmd.position.y = msg.pose.pose.position.y;
		        position_cmd.position.z = msg.pose.pose.position.z;
		        position_cmd.velocity.x = 0.0;
		        position_cmd.velocity.y = 0.0;
		        position_cmd.velocity.z = 0.0;
		        position_cmd.acceleration.x = 0.0;
		        position_cmd.acceleration.y = 0.0;
		        position_cmd.acceleration.z = 0.0;
		        position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
		        position_cmd.trajectory_id = traj_id_send;
			}
			break;


			default:
			cout<<"DEFAULT"<<endl;
			uav_state = 0; // GO BACK TO HOVER_STATE
			position_cmd.position.x = msg.pose.pose.position.x;
	        position_cmd.position.y = msg.pose.pose.position.y;
	        position_cmd.position.z = msg.pose.pose.position.z;
	        position_cmd.velocity.x = 0.0;
	        position_cmd.velocity.y = 0.0;
	        position_cmd.velocity.z = 0.0;
	        position_cmd.acceleration.x = 0.0;
	        position_cmd.acceleration.y = 0.0;
	        position_cmd.acceleration.z = 0.0;
	        position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
	        position_cmd.trajectory_id = traj_id_send;
			break;
		}

		position_cmd_pub.publish(position_cmd);
	}
}

void marker_pose_callback(const geometry_msgs::PoseStampedConstPtr &cmsg)
{
	//cout<<"marker_callback"<<endl;
	Vector3d p_cam(cmsg->pose.position.x, cmsg->pose.position.y, cmsg->pose.position.z);
	double cur_t = cmsg->header.stamp.toSec();
	//cout<<"cam pose: "<<p_cam.transpose()<<endl;

	if (p_cnt >= _NUM_P-1)
	{
		tar_obs.push_back(make_pair(cur_t, p_cam));
		//cout<<"time: "<<tar_obs.back().first-tar_obs.front().first<<endl;
		
		if (tar_obs.back().first-tar_obs.front().first<1.2)
		{
			Vector3d temp_marker_pose(0, 0, 0);

			for (list<pair<double, Vector3d>>::const_iterator it=tar_obs.begin(), end=tar_obs.end() ; it!=end; ++it)
				temp_marker_pose += it->second;

			//TODO valid marker pose
			//TODO marker_hover
			marker_pose = temp_marker_pose/_NUM_P;
			//cout<<"move: "<<(tar_obs.back().second.head<2>()-tar_obs.front().second.head<2>()).norm()<<endl;

			if ((tar_obs.back().second.head<2>()-tar_obs.front().second.head<2>()).norm()>0.1)
			{
				cout<<"DETECT TARGET MOVEMENT"<<endl;
				estimate_T(tar_obs);
				marker_pose = tar_obs.back().second;
				target_hover = false;
			}
			
			else
			{
				cout<<"TARGET STILL"<<endl;
				//marker_hover = marker_pose;
				target_hover = true;
				des_body_hover = marker_pose;
				//cout<<"marker_pose: "<<marker_pose.transpose()<<endl;
				des_body_hover(0) -= 1.5;
			}
		}

		tar_obs.pop_front();
		tar_obs.pop_front();
		p_cnt = _NUM_P-2;
	}
	else
	{
		tar_obs.push_back(make_pair(cur_t, p_cam));
		p_cnt++;
	}

	is_init = true;
}


void trigger_callback(const geometry_msgs::PoseStamped::ConstPtr &trigger_msg)
{
    if (is_init)
    {
        ROS_INFO("TRAJECTORY TRIGGERED");
        traj_id_send += 1; // trigger_msg->header.seq + 1;
        is_triggered = true;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "line");
    ros::NodeHandle nh("~");

    body_pose_sub = nh.subscribe("body_pose", 100, body_pose_callback);
    trigger_sub = nh.subscribe( "trigger", 100, trigger_callback);
    marker_pose_sub = nh.subscribe("marker_pose", 100, marker_pose_callback);

    tar_traj_pub= nh.advertise<visualization_msgs::Marker>("traj_viz", 1);
    plan_traj_pub= nh.advertise<visualization_msgs::Marker>("f_viz", 1);
    tar_pt_pub = nh.advertise<visualization_msgs::Marker>("pt_viz", 1);
    position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
/*
    tar_traj_pub = nh.advertise<visualization_msgs::Marker>("traj_viz", 1);
    plan_traj_pub = nh.advertise<visualization_msgs::Marker>("f_viz", 1);
    tar_pt_pub = nh.advertise<visualization_msgs::Marker>("pt_viz", 1);

	message_filters::Subscriber<geometry_msgs::PoseStamped> marker_pose_sub(nh, "marker_pose", 30);
	message_filters::Subscriber<nav_msgs::Odometry> body_pose_sub(nh, "body_pose", 100);

	typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), marker_pose_sub, body_pose_sub);
	sync.registerCallback(boost::bind(&est_callback, _1, _2));
*/
    ros::spin();
    return 0;
}