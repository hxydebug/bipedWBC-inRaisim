#ifndef __SWING_LEG_CONTROL_H
#define __SWING_LEG_CONTROL_H

#include "gait_generator.h"
#include "control.h"
#include "adaptivefootPlace.h"


class swing_leg_controller{
public:
    swing_leg_controller(robot *bike,gait_generator *gait_generator,FootHoldPlanner *footplanner, float desired_speed);
    void update(float current_time);
    Eigen::VectorXd get_action(Eigen::VectorXd user_cmd);
    void set_PDGain();
    Eigen::VectorXd tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT);
    Position postarget[2];
    float desired_xspeed;
    Eigen::Vector3d foot_position_now[2];
    FootHoldPlanner *foot_planner;
private:
    Eigen::VectorXd pGain,dGain;
    gait_generator *_gait_generator;
    robot *licycle;
    std::vector<int> last_leg_state = {0,0};
    Position phase_switch_foot_local_position[2];
    Position phase_switch_foot_local_position1[2];
    Eigen::Vector3d foot_position_begin[2];
    Eigen::Vector3d pos_com_last;
    Eigen::VectorXd _desired_height;
    Eigen::VectorXd bias_positions[2];
    Eigen::VectorXd angles;
    Eigen::VectorXd angles1;
    Eigen::VectorXd anglesV;
    Eigen::VectorXd action;

};


float gen_parabola(float phase, float start, float mid, float end);
Position gen_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos);
Eigen::VectorXd pd_tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT, float p_num, float d_num);

// add the cycloid
Eigen::VectorXd simple_cal_p(float p_start, float p_end, float period, float t_whole, bool isZ);
Position get_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos);
Eigen::Vector3d get_swing_foot_trajectory(float input_phase, Eigen::Vector3d start_pos, Eigen::Vector3d end_pos);
Position get_swing_foot_trajectory1(float input_phase, Position start_pos, Position end_pos);

#endif