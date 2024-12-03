#include "leg_controller.h"
#include "main.h"
#include <algorithm>


float a0[2][3] = {0};
float a1[2][3] = {0};
float a2[2][3] = {0};
float a3[2][3] = {0};

double rollMax = 2*PII/180;

float timer = 0;

Position init_pos[2];
Angle init_angle[2];

leg_controller::leg_controller(robot *biped,gait_generator *gait_gen,swing_leg_controller *swc,stance_leg_controller *stc){
  
	biped_robot = biped;
	leg_controller::set_PDGain();
  posT.resize(6);
  angT.resize(6);
  Tau_e.resize(6);
  stc_tau.resize(6);
  swc_tau.resize(6);
  set_xyz(fl,&init_angle[0],0-detx,0.08+0.06,-0.2-detz);
  set_xyz(fr,&init_angle[1],0-detx,-0.08-0.06,-0.2-detz);
  // set_xyz(fl,&init_angle[0],0.05,0.08,-0.32);
  // set_xyz(fr,&init_angle[1],0.05,-0.08,-0.32);
  posT << init_angle[0].q[0],init_angle[0].q[1],init_angle[0].q[2],
          init_angle[1].q[0],init_angle[1].q[1],init_angle[1].q[2];
  angT << 0,0,0,0,0,0;

  // init_angle[0].q[0] = 0.2;
  // init_angle[0].q[1] = 0.2;
  // init_angle[0].q[2] = 0.2;
  // init_angle[1].q[0] = 0.2;
  // init_angle[1].q[1] = 0.2;
  // init_angle[1].q[2] = 0.2;

  Kinematics(&init_angle[0],&init_pos[0],0);
  Kinematics(&init_angle[1],&init_pos[1],1);

  std::cout<<init_pos[0].x<<init_pos[0].y<<init_pos[0].z<<std::endl;
  std::cout<<init_pos[1].x<<init_pos[1].y<<init_pos[0].z<<std::endl;


  gait_generate = gait_gen;
  swctr = swc;
  stctr = stc;

}
void leg_controller::set_PDGain(){
	pGain.resize(6);
	dGain.resize(6);
	pGain.setConstant(20.0);
	dGain.setConstant(0);

}

Eigen::VectorXd leg_controller::control_body_directly(Eigen::Vector3d p_com_des, Eigen::Vector3d w_com_des, Eigen::Vector3d dp_com_des, Eigen::Vector3d dw_com_des){
  Eigen::VectorXd p_com = biped_robot->get_p_com();
	Eigen::Matrix3d com_rotm = biped_robot->get_com_rotmatrix();
	Eigen::VectorXd dp_com = biped_robot->get_dp_com();
	Eigen::VectorXd dw_com = biped_robot->get_dw_com();
  // std::cout<<"height:"<<p_com[2]<<std::endl;
  // std::cout<<"velocity:"<<dp_com.transpose()<<std::endl;
  Eigen::Matrix3d com_rotm_des = rpy2romatrix(w_com_des[0],w_com_des[1],w_com_des[2]);
  Eigen::Vector3d kp_p(0,0,0);
  Eigen::Vector3d kd_p(0,0,0);
  // Eigen::Vector3d kp_w(1500,1500,1500);
  // Eigen::Vector3d kd_w(50,50,50);
  Eigen::Vector3d kp_w(1500,1500,1500);
  Eigen::Vector3d kd_w(50,50,50);

  Eigen::Matrix3d M_kp_p = kp_p.asDiagonal();
  Eigen::Matrix3d M_kd_p = kd_p.asDiagonal();
  Eigen::Matrix3d M_kp_w = kp_w.asDiagonal();
  Eigen::Matrix3d M_kd_w = kd_w.asDiagonal();

  Eigen::VectorXd tau(6);
  Eigen::Matrix3d R_error = com_rotm_des * com_rotm.transpose();
  Eigen::AngleAxisd axis_angle = romatrix2AngleAxis(R_error);
  Eigen::Vector3d w_error = axis_angle.angle()*axis_angle.axis();
  // std::cout<<"angle_axis:"<<w_error<<std::endl;
  Eigen::Vector3d f_pd = M_kp_p * (p_com_des-p_com) + M_kd_p * (dp_com_des-dp_com);
  Eigen::Vector3d tau_pd = M_kp_w * w_error + M_kd_w * (dw_com_des - dw_com);

  tau << f_pd, tau_pd;
  return tau;
}
Eigen::VectorXd leg_controller::control_body_directly2(Eigen::Vector3d p_com_des, Eigen::Vector3d w_com_des, Eigen::Vector3d dp_com_des, Eigen::Vector3d dw_com_des){
  Eigen::VectorXd p_com = biped_robot->get_p_com();
	Eigen::Matrix3d com_rotm = biped_robot->get_com_rotmatrix();
	Eigen::VectorXd dp_com = biped_robot->get_dp_com();
	Eigen::VectorXd dw_com = biped_robot->get_dw_com();
  // std::cout<<"height:"<<p_com[2]<<std::endl;
  // std::cout<<"velocity:"<<dp_com.transpose()<<std::endl;
  Eigen::Matrix3d com_rotm_des = rpy2romatrix(w_com_des[0],w_com_des[1],w_com_des[2]);
  Eigen::Vector3d kp_p(0,0,0);
  Eigen::Vector3d kd_p(0,0,0);
  // Eigen::Vector3d kp_w(1500,1500,1500);
  // Eigen::Vector3d kd_w(50,50,50);
  Eigen::Vector3d kp_w(0,0,0);
  Eigen::Vector3d kd_w(20,20,20);

  Eigen::Matrix3d M_kp_p = kp_p.asDiagonal();
  Eigen::Matrix3d M_kd_p = kd_p.asDiagonal();
  Eigen::Matrix3d M_kp_w = kp_w.asDiagonal();
  Eigen::Matrix3d M_kd_w = kd_w.asDiagonal();

  Eigen::VectorXd tau(6);
  Eigen::Matrix3d R_error = com_rotm_des * com_rotm.transpose();
  Eigen::AngleAxisd axis_angle = romatrix2AngleAxis(R_error);
  Eigen::Vector3d w_error = axis_angle.angle()*axis_angle.axis();
  // std::cout<<"angle_axis:"<<w_error<<std::endl;
  Eigen::Vector3d f_pd = M_kp_p * (p_com_des-p_com) + M_kd_p * (dp_com_des-dp_com);
  Eigen::Vector3d tau_pd = M_kp_w * w_error + M_kd_w * (dw_com_des - dw_com);

  tau << f_pd, tau_pd;
  return tau;
}
Eigen::VectorXd leg_controller::get_action(int Run_mode,Eigen::VectorXd user_cmd){
		
  if(Run_mode==0){ 
    // posT<< 0.5,0.3,-0.3,-0.5,-0.3,-0.3;
    // angT<< 0,0,0,0,0,0;
    // set_xyz(fl,&init_angle[0],0.16,0.08,-0.12);
    // set_xyz(fr,&init_angle[1],0.16,-0.08,-0.12);
    // set_xyz(fl,&init_angle[0],0,0.08,-0.32);
    // set_xyz(fr,&init_angle[1],0,-0.08,-0.32);
    posT << init_angle[0].q[0],init_angle[0].q[1],init_angle[0].q[2],
            init_angle[1].q[0],init_angle[1].q[1],init_angle[1].q[2];
    // posT << 0.2,0.2,0.2,
    //         0.2,0.2,0.2;
    pGain.setConstant(180.0);
	  dGain.setConstant(2);

    Tau_e = leg_controller::tau(biped_robot->get_leg_pos(),biped_robot->get_leg_vel(),posT,angT);
  }
  else if(Run_mode==1){ 
    float t = timer;
    set_xyz(fl,&init_angle[0],0.1*sin(2*PII/2*t)-detx,0.08+0.06,0.1*cos(2*PII/2*t)-0.3-detz);
    // set_xyz(fr,&init_angle[1],0.1*sin(2*PII/2*t),-0.08,0.1*cos(2*PII/2*t)-0.3);
    posT << init_angle[0].q[0],init_angle[0].q[1],init_angle[0].q[2],
            init_angle[1].q[0],init_angle[1].q[1],init_angle[1].q[2];
    // posT << 0.2,0.2,0.2,
    //         0.2,0.2,0.2;
    pGain.setConstant(180.0);
	  dGain.setConstant(2);

    Tau_e = leg_controller::tau(biped_robot->get_leg_pos(),biped_robot->get_leg_vel(),posT,angT);
    timer += 0.001;
  }
  else{
    // gait generator
    gait_generate->update(timer);

    // ground reaction force calculate    ***** it can be put in another pthread *****
    stc_tau = stctr->get_action(user_cmd);
    // Eigen::VectorXd stc_tau(6);
    // stc_tau.setConstant(0);

    // position controller
    swctr->update(timer);
    swc_tau = swctr->get_action(user_cmd);

    Eigen::VectorXd ltau(3),rtau(3);
    if(gait_generate->leg_state[0]==stance_leg || gait_generate->leg_state[0]==Early_Contact){
      ltau = swc_tau.head(3)+stc_tau.head(3);
      // ltau <<0,0,0;
    }
    else{
      ltau = swc_tau.head(3);
      // ltau <<0,0,0;
    }
    if(gait_generate->leg_state[1]==stance_leg || gait_generate->leg_state[1]==Early_Contact){
      rtau = swc_tau.tail(3)+stc_tau.tail(3);
      // rtau <<0,0,0;
    }
    else{
      rtau = swc_tau.tail(3);
      // rtau <<0,0,0;
    }

    Tau_e << ltau,rtau;

    timer += 0.001;

  }

  for(int i(0);i<6;i++){
		if(Tau_e[i] < -48.0) Tau_e[i] = -48.0;
		if(Tau_e[i] > 48.0) Tau_e[i] = 48.0;
	}

  return Tau_e;

}

void leg_controller::dataBusWrite(DataBus &robotState){
  robotState.Fr_ff = Eigen::VectorXd::Zero(6);
  if(gait_generate->leg_state[1] == 0){
    robotState.legState = DataBus::LegState::LSt;
    robotState.swing_fe_pos_des_W = swctr->foot_position_now[1];
    robotState.stance_fe_pos_cur_W=robotState.fe_l_pos_W;
    robotState.stance_fe_rot_cur_W=robotState.fe_l_rot_W;
    // robotState.Fr_ff[2] = -103;
  } 
  else {
    robotState.legState = DataBus::LegState::RSt;
    robotState.swing_fe_pos_des_W = swctr->foot_position_now[0];
    robotState.stance_fe_pos_cur_W=robotState.fe_r_pos_W;
    robotState.stance_fe_rot_cur_W=robotState.fe_r_rot_W;
    // robotState.Fr_ff[5] = -103;
  }
  
  
  robotState.Fr_ff = -stctr->GRF;


}

Eigen::VectorXd leg_controller::final_tau(DataBus &robotState){
  // pGain << 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0;
  // dGain << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
  pGain.setConstant(180.0);
	dGain.setConstant(2.0);

  // Eigen::VectorXd Tau = leg_controller::tau(biped_robot->get_leg_pos(),biped_robot->get_leg_vel(),robotState.motors_posDes,robotState.motors_velDes);
  // // assign swing and stance foot
  // Eigen::VectorXd ltau(3),rtau(3);
  // if(gait_generate->leg_state[0]==stance_leg || gait_generate->leg_state[0]==Early_Contact){
  //   ltau = swc_tau.head(3)+stc_tau.head(3);
  //   // ltau <<0,0,0;
  // }
  // else{
  //   ltau = swc_tau.head(3);
  //   // ltau <<0,0,0;
  // }
  // if(gait_generate->leg_state[1]==stance_leg || gait_generate->leg_state[1]==Early_Contact){
  //   rtau = swc_tau.tail(3)+stc_tau.tail(3);
  //   // rtau <<0,0,0;
  // }
  // else{
  //   rtau = swc_tau.tail(3);
  //   // rtau <<0,0,0;
  // }

  Eigen::VectorXd Tau = swc_tau + robotState.motors_torDes;
  // Tau += robotState.motors_torDes;
  for(int i(0);i<6;i++){
    if(Tau[i] < -48.0) Tau[i] = -48.0;
    if(Tau[i] > 48.0) Tau[i] = 48.0;
	}

  return Tau;
}
Eigen::VectorXd leg_controller::tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT){
  
  return dGain.cwiseProduct(vT-vA) + pGain.cwiseProduct(pT-pA);

}


void leg_controller::goto_xyz(float xx,float yy,float zz,Leg leg){

  //define the variables
  Angle now_angle;
  Angle now_angleV;
  Position now_position;
  Position now_velocity;

  //get the joint angle position and velocity
  auto pos = biped_robot->get_leg_pos();
  if(leg == fl){
    for(int i(0);i<3;i++){
        now_angle.q[i] = pos[i];
    }
  }
  else{
    for(int i(0);i<3;i++){
        now_angle.q[i] = pos[i+3];
    }
  }

  auto vel = biped_robot->get_leg_vel();
  if(leg == fl){
    for(int i(0);i<3;i++){
        now_angleV.q[i] = vel[i];
    }
  }
  else{
    for(int i(0);i<3;i++){
        now_angleV.q[i] = vel[i+3];
    }
  }

  //get the end point position and velocity using forward kinematics and jacobian
  Kinematics(&now_angle,&now_position,leg);

  now_velocity.x = 0;
  now_velocity.y = 0;
  now_velocity.z = 0;

  //input the desire position and velocity
	Position pdes;
  Position vdes;

  pdes.x = xx;
  pdes.y = yy;
  pdes.z = zz;

  vdes.x = 0;
  vdes.y = 0;
  vdes.z = 0;

  //init chabu
  init_chabu(&pdes,&vdes,&now_position,&now_velocity,period,leg);

  
}

void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz){
  Position pos;
  pos.x = xx;
  pos.y = yy;
  pos.z = zz;

  Inv_kinematics_ref(angle,&pos,leg);
}

void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float step,Leg leg){
	
	float p0[3];
	float pf[3];
	float v0[3];
	float vf[3];
	float tf;
	
	p0[0] = pini->x;
	p0[1] = pini->y;
	p0[2] = pini->z;
	
	pf[0] = pdes->x;
	pf[1] = pdes->y;
	pf[2] = pdes->z;
	
	v0[0] = vini->x;
	v0[1] = vini->y;
	v0[2] = vini->z;
	
	vf[0] = vdes->x;
	vf[1] = vdes->y;
	vf[2] = vdes->z;

	tf = step;
	
	for(int i=0;i<3;i++){
		a0[leg][i] = p0[i];
		a1[leg][i] = v0[i];
		a2[leg][i] = 3.0/(tf*tf)*(pf[i]-p0[i])-2.0/tf*v0[i]-1.0/tf*vf[i];
		a3[leg][i] = -2.0/(tf*tf*tf)*(pf[i]-p0[i])+1.0/(tf*tf)*(vf[i]+v0[i]);
	}
}

void chabu(Position *pos,float step,Leg leg){
	float p[3];
	for(int i=0;i<3;i++){
		p[i] = a0[leg][i] + a1[leg][i]*step + a2[leg][i]*step*step +a3[leg][i]*step*step*step;
	}
	pos->x = p[0];
	pos->y = p[1];
	pos->z = p[2];
	
}

float linear(float ini,float des,float tf,float step){
	float p0 = ini;
	float pf = des;
	float b0,b1;
	b0 = p0;
	b1 = (pf-p0)/tf;
	float p = b0+b1*step;
	return p;
}

