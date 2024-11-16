#ifndef __MODEL_H
#define __MODEL_H

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "main.h"
#include<iostream>
#include<queue>

class robot{

public:
	robot(raisim::ArticulatedSystem* client);
	raisim::ArticulatedSystem* biped;

	int getGeneralizedCoordinateDim();
	int getDOF();
	void initial_robot();
	Eigen::VectorXd get_p_com();
	Eigen::Matrix3d get_com_rotmatrix();
	Eigen::VectorXd get_dp_com();
	Eigen::VectorXd get_dw_com();
	Eigen::VectorXd get_leg_vel();
	Eigen::VectorXd get_leg_pos();
	Eigen::VectorXd get_base_orientation();
	Eigen::VectorXd get_base_rpy();
	Eigen::VectorXd get_base_angle_v();
	double get_base_v();
	Eigen::Vector3d get_base_vec();
	double get_psi();
	double get_varphi();
	double get_dpsi();
	double get_dvarphi();
	void update_state();
	void update_angle();
	std::vector<int> GetFootContact();
	void step(Eigen::VectorXd leg_tau,Eigen::VectorXd body_tau);
	void dataBusWrite(DataBus &busIn);
private:
	Eigen::VectorXd pos;
	Eigen::VectorXd vel;
	Eigen::Matrix3d rot_matrix;
	double dpsi,last_dpsi = 0,llast_dpsi = 0;
	double dvarphi,last_dvarphi = 0,llast_dvarphi = 0;
	std::vector<int> contact_leg={0,0};
	// std::queue<double> dpsi_q;
	// std::queue<double> dvarphi_q;

};

#endif