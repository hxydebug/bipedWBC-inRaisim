#ifndef __LEG_CONTROL_H
#define __LEG_CONTROL_H


#include "model.h"
#include "control.h"
#include "swing_leg_controller.h"
#include "stance_leg_controller.h"

#define period 300

class leg_controller{

public:
	leg_controller(robot *biped,gait_generator *gait_gen,swing_leg_controller *swc, stance_leg_controller *stc);
	Eigen::VectorXd control_body_directly(Eigen::Vector3d p_com_des, Eigen::Vector3d w_com_des, Eigen::Vector3d dp_com_des, Eigen::Vector3d dw_com_des);
	Eigen::VectorXd control_body_directly2(Eigen::Vector3d p_com_des, Eigen::Vector3d w_com_des, Eigen::Vector3d dp_com_des, Eigen::Vector3d dw_com_des);
	void set_PDGain();
	Eigen::VectorXd tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT);
	Eigen::VectorXd get_action(int Run_mode,Eigen::VectorXd user_cmd);
	void create_gait(void);
	void goto_xyz(float xx,float yy,float zz,Leg leg);
private:
	robot *biped_robot;
	Eigen::VectorXd pGain,dGain;
	double T = 0.2;
	float dt = 0.001;
	float Hf = 0.1;
	float Hb = 0.305;
	float swing_time = 0.1;

	float tf = 0;
	float p0 = 0;
	float pf = 0;
	float vf = 0;
	float v0 = 0;

	Angle angle[2];
	Angle angleV[2];
	Position position[2];
	Position velocity[2];

	int force_flag = 0;

	Eigen::VectorXd posT,angT;
	Eigen::VectorXd Tau_e,Tau_l,Tau_r;

	std::vector<float> x_position;
	std::vector<float> z_position;

	gait_generator *gait_generate;
 	swing_leg_controller *swctr;
	stance_leg_controller *stctr;

};


void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float step,Leg leg);
void chabu(Position *pos,float step,Leg leg);
float linear(float ini,float des,float tf,float step);

void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz);


#endif