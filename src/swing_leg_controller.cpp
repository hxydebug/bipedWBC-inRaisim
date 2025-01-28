#include "swing_leg_controller.h"

float _KP = 0.05;
float foot_clearance = 0.01;
float desired_height = 0.565;
float t_swing = stance_t;
Eigen::Vector3d dP;

swing_leg_controller::swing_leg_controller(robot *bike,gait_generator *gait_generator,FootHoldPlanner *footplanner, float desired_speed){
  licycle = bike;
  _gait_generator = gait_generator;
  foot_planner = footplanner;

  postarget[0].x = 0;
  postarget[0].y = 0;
  postarget[0].z = 0;
  postarget[1].x = 0;
  postarget[1].y = 0;
  postarget[1].z = 0;

  last_leg_state = _gait_generator->desired_leg_state;
  phase_switch_foot_local_position[0] = getFootPositionInBaswFrame(licycle->get_leg_pos(),0);
  phase_switch_foot_local_position[1] = getFootPositionInBaswFrame(licycle->get_leg_pos(),1);
  phase_switch_foot_local_position1[0] = getFootPositionInBaswFrame(licycle->get_leg_pos(),0);
  phase_switch_foot_local_position1[1] = getFootPositionInBaswFrame(licycle->get_leg_pos(),1);
  pos_com_last = licycle->get_p_com();
  desired_xspeed = desired_speed;
  angles.resize(6);
  angles1.resize(6);
  anglesV.resize(6);
  _desired_height.resize(3);
  action.resize(6);
  bias_positions[0].resize(3);
  bias_positions[1].resize(3);
  _desired_height << 0,0,desired_height-foot_clearance;
  angles.setConstant(0);
  anglesV.setConstant(0);
  action.setConstant(0);
  bias_positions[0] << 0.001,wid/2+0.03,0;
  bias_positions[1] << 0.001,-wid/2-0.03,0;
  swing_leg_controller::set_PDGain();

}

void swing_leg_controller::update(float current_time){
  std::vector<int> new_leg_state = _gait_generator->desired_leg_state;
  
  // Detects phase switch for each leg so we can remember the feet position at
  // the beginning of the swing phase.
  for(int i(0);i<2;i++){
    if (new_leg_state[i]==swing_leg && new_leg_state[i] != last_leg_state[i]) {
      phase_switch_foot_local_position[i] = getFootPositionInBaswFrame(licycle->get_leg_pos(),i);
      pos_com_last = licycle->get_p_com();
      Eigen::Vector3d foot_position_body;
      foot_position_body << phase_switch_foot_local_position[i].x, phase_switch_foot_local_position[i].y, phase_switch_foot_local_position[i].z;
      Eigen::Matrix3d com_rotm = licycle->get_com_rotmatrix();
      foot_position_begin = com_rotm * foot_position_body + pos_com_last;
    }
  }

  last_leg_state = new_leg_state;

}

Eigen::VectorXd swing_leg_controller::get_action(Eigen::VectorXd user_cmd){

  Eigen::VectorXd com_velocity(3);
  com_velocity = licycle->get_dp_com();
  com_velocity[2] = 0;
  // com_velocity << 0,0,0;
  // std::cout<<"velocity:"<<licycle->get_base_v()<<std::endl;
  Eigen::Matrix3d com_rotm = licycle->get_com_rotmatrix();
  Eigen::VectorXd p_com = licycle->get_p_com();
  Eigen::VectorXd pos = licycle->get_leg_pos();
  Eigen::VectorXd dw_com = licycle->get_dw_com();
  Eigen::VectorXd w_com = licycle->get_base_rpy();
  Eigen::Vector3d legpos[2];
  legpos[0] = pos.head(3);
  legpos[1] = pos.tail(3);
  Eigen::VectorXd vel = licycle->get_leg_vel();
  Eigen::Vector3d legvel[2];
  legvel[0] = vel.head(3);
  legvel[1] = vel.tail(3);

  Eigen::VectorXd leg_motor_torque(6);
  leg_motor_torque.setConstant(0);
  Eigen::Vector3d motor_torque[2];


  Eigen::VectorXd linear_velocity = com_velocity;
  Eigen::VectorXd desired_linear_velocity(3);
  desired_linear_velocity << user_cmd[0],user_cmd[1],0;
  // // command is in the body frame
  // Eigen::Vector3d v_bd;
  // v_bd << user_cmd[0],user_cmd[1],0;
  // // make command in z is zero
  // desired_linear_velocity = com_rotm * v_bd;
  // desired_linear_velocity[2] = 0;
  // considering rotation for foot placement
  double desired_dw_com = user_cmd[3];
  desired_height = user_cmd[2];
  _desired_height << 0,0,desired_height-foot_clearance;
  double feetR[2], feetInitAngle[2];
  feetR[0] = wid/2 + Len0-0.06;
  feetR[1] = wid/2 + Len0-0.06;
  feetInitAngle[0] = 83.5*PII/180.0;
  feetInitAngle[1] = -83.5*PII/180.0;

  // calculate position 
  for(int i(0);i<2;i++){
    Eigen::VectorXd bias_pos = com_rotm*bias_positions[i];
    bias_pos[2] = -bias_pos[2];
    if (_gait_generator->leg_state[i]==stance_leg || _gait_generator->leg_state[i]==Early_Contact)
    {

      Eigen::Vector3d angs;
      angs.setConstant(0);
      Eigen::Vector3d ansV;
      ansV.setConstant(0);
      // stance phase
      motor_torque[i] = pd_tau(legpos[i], legvel[i], angs, ansV, 0, 2);
    }
    else 
    {
      // get desired foot placement in world frame
      double rotation_angle_bias = 0.5 * dw_com[2] * _gait_generator->stance_duration[i]
                    + dw_com[2] * (1-_gait_generator->normalized_phase[i]) * _gait_generator->swing_duration[i]
                    + 0.005 * (dw_com[2] - desired_dw_com);
      double rotation_xpos_bias = feetR[i] * cos(w_com[2] + feetInitAngle[i] + rotation_angle_bias);
      double rotation_ypos_bias = feetR[i] * sin(w_com[2] + feetInitAngle[i] + rotation_angle_bias);
      Eigen::VectorXd foot_target_position = 0.5 * linear_velocity * _gait_generator->stance_duration[i]
                                           + linear_velocity * (1-_gait_generator->normalized_phase[i]) * _gait_generator->swing_duration[i]
                                           + _KP * (linear_velocity - desired_linear_velocity);
      foot_target_position[0] += rotation_xpos_bias + p_com[0];
      foot_target_position[1] += rotation_ypos_bias + p_com[1];

      // use adaptive foot placement
      int Nsteps = 5;
      double b0x = p_com[0] + com_velocity[0]/foot_planner->omega - foot_position_begin[0];
      double b0y = p_com[1] + com_velocity[1]/foot_planner->omega - foot_position_begin[1];
      foothold = foot_planner->ComputeNextfootHold(Nsteps,b0x,b0y,_gait_generator->leg_state[1],foot_position_begin[0],foot_position_begin[1],_gait_generator->normalized_phase[0]);
      foot_target_position[0] = foot_position_begin[0] + foothold[0];
      foot_target_position[1] = foot_position_begin[1] + foothold[1];
      // foothold_dcm[0] = foot_position_begin[0] + foothold[0];
      // foothold_dcm[1] = foot_position_begin[1] + foothold[1];
      // modify to desired height
      // foot_target_position[2] = 0;
      foot_target_position[2] = p_com[2]-desired_height;

      // foothold_dcm[2] = foot_target_position[2];
      // foothold_heuristic = foot_target_position;

      // get beginning foot position in world frame
      foot_position_now[i] = get_swing_foot_trajectory(_gait_generator->normalized_phase[i],foot_position_begin,foot_target_position);

      // from world to body frame
      Eigen::Vector3d foot_position = com_rotm.transpose() * (foot_position_now[i]-p_com);
      postarget[i].x = foot_position[0];
      postarget[i].y = foot_position[1];
      postarget[i].z = foot_position[2];
      Eigen::Vector3d Current_positionVector,Desired_positionVector;
      Desired_positionVector = foot_position;
      // get current position
      Position Current_positionxyz = getFootPositionInBaswFrame(licycle->get_leg_pos(),i);
      Current_positionVector << Current_positionxyz.x,Current_positionxyz.y,Current_positionxyz.z;
      // get current velocity
      Eigen::Vector3d Current_velocityVector = calcu_Jaco(legpos[i],i)*legvel[i];
      // get virtual force
      Eigen::Vector3d virtualForce = swing_leg_controller::tau(Current_positionVector,Current_velocityVector,Desired_positionVector,dP);
      motor_torque[i] = calcu_Jaco(legpos[i],i).transpose()*virtualForce;

    }
    // the torque of ith leg

  }
  leg_motor_torque << motor_torque[0], motor_torque[1];
  return leg_motor_torque;


}

void swing_leg_controller::set_PDGain(){
	pGain.resize(3);
	dGain.resize(3);
	pGain << 2000,2000,2000;
  dGain << 50,50,50;

}

Eigen::VectorXd swing_leg_controller::tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT){
  
  return dGain.cwiseProduct(vT-vA) + pGain.cwiseProduct(pT-pA);

}

Eigen::VectorXd pd_tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT, float p_num, float d_num){
  
  Eigen::VectorXd p_gain(3);
  Eigen::VectorXd d_gain(3);
  p_gain.setConstant(p_num);
  d_gain.setConstant(d_num);
  return d_gain.cwiseProduct(vT-vA) + p_gain.cwiseProduct(pT-pA);

}

float gen_parabola(float phase, float start, float mid, float end){
  /*** Gets a point on a parabola y = a x^2 + b x + c.

  The Parabola is determined by three points (0, start), (0.5, mid), (1, end) in
  the plane.

  Args:
    phase: Normalized to [0, 1]. A point on the x-axis of the parabola.
    start: The y value at x == 0.
    mid: The y value at x == 0.5.
    end: The y value at x == 1.

  Returns:
    The y value at x == phase.
  ***/
  float mid_phase = 0.5;
  float delta_1 = mid - start;
  float delta_2 = end - start;
  float delta_3 = mid_phase*mid_phase - mid_phase;
  float coef_a = (delta_1 - delta_2 * mid_phase) / delta_3;
  float coef_b = (delta_2 * mid_phase*mid_phase - delta_1) / delta_3;
  float coef_c = start;

  return coef_a * phase * phase + coef_b * phase + coef_c;
}

Position gen_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos){
  /*** Generates the swing trajectory using a parabola.

  Args:
    input_phase: the swing/stance phase value between [0, 1].
    start_pos: The foot's position at the beginning of swing cycle.
    end_pos: The foot's desired position at the end of swing cycle.

  Returns:
    The desired foot position at the current phase.
  ***/

  float phase = input_phase;
  if(input_phase <= 0.5) phase = 0.8 * sin(input_phase * PII);
  else phase = 0.8 + (input_phase - 0.5) * 0.4;

  Position pos;
  pos.x = (1 - phase) * start_pos.x + phase * end_pos.x;
  pos.y = (1 - phase) * start_pos.y + phase * end_pos.y;
  float max_clearance = 0.1;
  float mid = std::max(end_pos.z, start_pos.z) + max_clearance;
  pos.z = gen_parabola(phase, start_pos.z, mid, end_pos.z);

  return pos;
}

// add the cycloid
Eigen::VectorXd simple_cal_p(float p_start, float p_end, float period, float t_whole, bool isZ){

  period = period * t_whole;

  if (isZ) {
    t_whole = t_whole * 0.5;
  }
  float p_des = p_start + (p_end - p_start) * (period / t_whole - sin(2 * PII * period / t_whole) / (2 * PII));
  float v_des = (p_end - p_start) / t_whole * (1 - cos(2 * PII * period / t_whole));
  float a_des = 2 * PII * sin(2 * PII * period / t_whole)*(p_end - p_start) / (t_whole * t_whole);
  Eigen::Vector3d ans;
  ans << p_des, v_des, a_des;
  return ans;
}

Eigen::VectorXd simple_cal_p1(float p_start, float p_end, float period, float t_whole, bool isdown, float tt){

  period = period * t_whole;

  if (isdown) {
    t_whole = t_whole * (1-tt);
  }
  else{
    t_whole = t_whole * tt;
  }
  float p_des = p_start + (p_end - p_start) * (period / t_whole - sin(2 * PII * period / t_whole) / (2 * PII));
  float v_des = (p_end - p_start) / t_whole * (1 - cos(2 * PII * period / t_whole));
  float a_des = 2 * PII * sin(2 * PII * period / t_whole)*(p_end - p_start) / (t_whole * t_whole);
  Eigen::Vector3d ans;
  ans << p_des, v_des, a_des;
  return ans;
}
Position get_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos){
  float phase = input_phase;

  Eigen::Vector3d p_x;
  Eigen::Vector3d p_y;
  Eigen::Vector3d p_z;
  Position pos;

  p_x = simple_cal_p(start_pos.x, end_pos.x, phase, t_swing, false);
  p_y = simple_cal_p(start_pos.y, end_pos.y, phase, t_swing, false);

  float max_clearance = 0.1;
  float mid = std::max(end_pos.z, start_pos.z) + max_clearance;
  double tt = 0.5;
  if (phase < tt) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
    p_z = simple_cal_p1(start_pos.z, mid, phase, t_swing, false, tt);
  }
  else {
    p_z = simple_cal_p1(mid, end_pos.z, phase-tt, t_swing, true, tt);
  }
  pos.x = p_x[0];
  pos.y = p_y[0];
  pos.z = p_z[0];

  dP[0] = p_x[1];
  dP[1] = p_y[1];
  dP[2] = p_z[1];

  return pos;
}

Eigen::Vector3d get_swing_foot_trajectory(float input_phase, Eigen::Vector3d start_pos, Eigen::Vector3d end_pos){
  float phase = input_phase;

  Eigen::Vector3d p_x;
  Eigen::Vector3d p_y;
  Eigen::Vector3d p_z;
  Eigen::Vector3d pos;

  p_x = simple_cal_p(start_pos[0], end_pos[0], phase, t_swing, false);
  p_y = simple_cal_p(start_pos[1], end_pos[1], phase, t_swing, false);

  float max_clearance = 0.1;
  float mid = std::max(end_pos[2], start_pos[2]) + max_clearance;
  
  // if (phase < 0.5) {
  //   p_z = simple_cal_p(start_pos[2], mid, phase, t_swing, true);
  // }
  // else {
  //   p_z = simple_cal_p(mid, end_pos[2], phase-0.5, t_swing, true);
  // }
  float tt = 0.2;
  if (phase < tt) {
    p_z = simple_cal_p1(start_pos[2], mid, phase, t_swing, false, tt);
  }
  else {
    p_z = simple_cal_p1(mid, end_pos[2], phase-tt, t_swing, true, tt);
  }
  pos[0] = p_x[0];
  pos[1] = p_y[0];
  pos[2] = p_z[0];

  dP[0] = p_x[1];
  dP[1] = p_y[1];
  dP[2] = p_z[1];

  return pos;
}

Position get_swing_foot_trajectory1(float input_phase, Position start_pos, Position end_pos){

  float phase = input_phase;

  Position pos;
  pos.x = (1 - phase) * start_pos.x + phase * end_pos.x;
  pos.y = (1 - phase) * start_pos.y + phase * end_pos.y;
  pos.z = (1 - phase) * start_pos.z + phase * end_pos.z;

  return pos;
}