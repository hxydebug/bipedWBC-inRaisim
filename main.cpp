#include "main.h"
#include "leg_controller.h"
#include "model.h"
#include "swing_leg_controller.h"
#include "stance_leg_controller.h"
#include <iostream>
#include <fstream>
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include "wbc_priority.h"
#include "useful_math.h"
#include "bikebot_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include "adaptivefootPlace.h"
using namespace std;

int main(int argc, char* argv[]) {

  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World::setActivationKey("/home/hxy/.raisim/activate.raisim");
  raisim::World world;
  auto Biped = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\biped.urdf");
  // auto smart_bycicle = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\Smart Bikeurdf\\urdf\\unmanned_bicycle.urdf");
 
  auto ground = world.addGround(0,"steel");

  world.setMaterialPairProp("steel", "rubber", 0.45, 0.15, 0.001);
  cout << Biped->getDOF() << Biped->getGeneralizedCoordinateDim() << endl;
  world.setTimeStep(0.001);

  // Eigen::VectorXd jointNominalConfig(Biped->getGeneralizedCoordinateDim()), jointVelocityTarget(Biped->getDOF());
  // jointNominalConfig<<  0, 0, 0.38, RpyToqua(0.0*PII/180.0,0.0*PII/180.0,0.0*PII/180.0),
  //                       0,0,0,
  //                       0,0,0;

  // jointVelocityTarget.setZero();
  // jointVelocityTarget[0] = 0.0;//velocity
  // Biped->setGeneralizedCoordinate(jointNominalConfig);
  // Biped->setGeneralizedVelocity(jointVelocityTarget);
  // Biped->setGeneralizedForce(Eigen::VectorXd::Zero(Biped->getDOF()));
  // Biped->setName("Biped");

  /// smart_bycicle init
  robot robot(Biped);

  //生成数据编号
  char ch[64] = {0};
  time_t tt = time(NULL);
  strftime(ch, sizeof(ch) - 1, "%H%M", localtime(&tt));
  char result[100] = {0};
  sprintf(result, "/home/hxy/sim_data1/dataFile%s.txt", ch);
  std::ofstream dataFile;
  dataFile.open(result, std::ofstream::app);

  ///controller init
  gait_generator gait_gen(&robot);
  FootHoldPlanner footplanner(0.45, 0.25, 0.5, 0.12);
 	swing_leg_controller swc(&robot,&gait_gen,&footplanner,0);
 	stance_leg_controller stc(&robot,&gait_gen,0);
  leg_controller l_control(&robot,&gait_gen,&swc,&stc);
  Pin_KinDyn kinDynSolver("rsc/biped.urdf"); // kinematics and dynamics solver
  DataBus RobotState(kinDynSolver.model_nv); // data bus
  WBC_priority WBC_solv(kinDynSolver.model_nv, 12, 16, 0.45, 0.001); // WBC solver

  /// launch raisim server for visualization. Can be visualized using raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();

  Eigen::VectorXd leg_tau;
  Eigen::VectorXd body_tau;
  Eigen::VectorXd user_cmd(4),interface_cmd(4);
  float global_timer = 0;

  user_cmd<< 0.2,0.0,0.45,0.0;   //vx,vy,height,dyaw
  interface_cmd = user_cmd;   //vx,vy,height,dyaw
  double x_com_desire=0.0;
  double y_com_desire=0.0;
  double yaw_desire=0.0;
  
  // calculate running time for wbc
  Timer t;

  while(1){
    raisim::MSLEEP(5);
    t.start();
    // get sensor data
    robot.update_state();
    robot.dataBusWrite(RobotState);

    // update kinematics and dynamics info
    kinDynSolver.dataBusRead(RobotState);
    kinDynSolver.computeJ_dJ();
    kinDynSolver.computeDyn();
    kinDynSolver.dataBusWrite(RobotState);

    RobotState.motionState = DataBus::Walk; 
    
    Eigen::Vector3d p_com_des,w_com_des,dp_com_des,dw_com_des;
    p_com_des<<0,0,0.45;//0.41~0.42
    dp_com_des<<0,0,0;
    w_com_des<<0,0,0;
    dw_com_des<<0,0,0;
    body_tau = l_control.control_body_directly(p_com_des, w_com_des, dp_com_des, dw_com_des);
    if(global_timer>0.5){
      body_tau << 0,0,0,0,0,0;
    }
    // if(global_timer>2)  body_tau = l_control.control_body_directly2(p_com_des, w_com_des, dp_com_des, dw_com_des);
    // if(global_timer>5)  
    // body_tau << 0,0,0,0,0,0;
    // x_com_desire +=  user_cmd[0] * 0.001;
    // y_com_desire +=  user_cmd[1] * 0.001;
    // yaw_desire += user_cmd[3] * 0.001;
    interface_cmd = user_cmd;
    // double kp = 1;
    // interface_cmd[0] += kp*(x_com_desire-RobotState.q(0));
    // interface_cmd[1] += kp*(y_com_desire-RobotState.q(1));
    // leg_tau = l_control.get_action(2,interface_cmd);
    // l_control.dataBusWrite(RobotState);
    // cout<<footplanner.leftoverTime<<endl;
    // ------------- WBC ------------
    // WBC input
    RobotState.des_ddq = Eigen::VectorXd::Zero(kinDynSolver.model_nv);
    RobotState.des_dq = Eigen::VectorXd::Zero(kinDynSolver.model_nv);
    RobotState.des_delta_q = Eigen::VectorXd::Zero(kinDynSolver.model_nv);
    RobotState.base_rpy_des << 0, 0, yaw_desire;
    RobotState.base_pos_des << x_com_desire, y_com_desire, user_cmd[2];

    // adjust des_delata_q, des_dq and des_ddq to achieve forward walking
    // float g_h = 9.8/0.45;
    // interface_cmd[0] = RobotState.dq(0) + 0.001 * g_h*(RobotState.q(0)-swc.stance_foot_pos[0]);
    // interface_cmd[1] = RobotState.dq(1) + 0.001 * g_h*(RobotState.q(1)-swc.stance_foot_pos[1]);
    leg_tau = l_control.get_action(2,interface_cmd);
    l_control.dataBusWrite(RobotState);
    if (global_timer>0.5) {
        RobotState.des_delta_q.block<2, 1>(0, 0) << RobotState.dq(0) *0.001, RobotState.dq(1) * 0.001;
        RobotState.des_delta_q(5) = interface_cmd[3] * 0.001;
        RobotState.des_dq.block<2, 1>(0, 0) <<  interface_cmd[0] ,  interface_cmd[1] ;
        RobotState.des_dq(5) = interface_cmd[3];

        double k = 0;
        RobotState.des_ddq.block<2, 1>(0, 0) << k * (interface_cmd[0] - RobotState.dq(0)), k * (interface_cmd[1] -
                                                                                              RobotState.dq(1));
        RobotState.des_ddq(5) = k * (interface_cmd[3] - RobotState.dq(5));
    }

    // leg_tau << 0,0,0,0,0,0;
    // cout << RobotState.Fr_ff.transpose() <<endl;
    if (global_timer>0.5) {
      // WBC Calculation
      WBC_solv.dataBusRead(RobotState);
      WBC_solv.computeDdq(kinDynSolver);
      WBC_solv.computeTau();
      WBC_solv.dataBusWrite(RobotState);

      // get the final joint torque
      Eigen::VectorXd pos_des=kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
      RobotState.motors_posDes = pos_des.block(7,0, kinDynSolver.model_nv-6,1);
      RobotState.motors_velDes = RobotState.wbc_dq_final.tail(6);
      // RobotState.motors_velDes = Eigen::VectorXd::Zero(6);
      RobotState.motors_torDes = RobotState.wbc_tauJointRes;
      // cout<<RobotState.motors_posDes<<endl;
      // cout<<RobotState.motors_velDes<<endl;
      
      // cout<<global_timer<<endl;
      // cout<<RobotState.q(0)<<" "<<RobotState.q(1)<<" "<<RobotState.q(2)<<endl;
      Eigen::VectorXd leg_tau1 = l_control.final_tau(RobotState);
      // cout<<leg_tau1<<endl;
      robot.step(leg_tau1,body_tau);
    }
    else{
      robot.step(leg_tau,body_tau);
    }
    float wbc_time = (float)t.getSeconds();
    // cout << "wbc_time: " << wbc_time << endl;
    // fix test 
    // Eigen::VectorXd jointNominalConfig(Biped->getGeneralizedCoordinateDim()), jointVelocityTarget(Biped->getDOF());
    // jointNominalConfig<<  0, 0, 0.6, RpyToqua(0.0*PII/180.0,0.0*PII/180.0,0.0*PII/180.0),
    //                     0,0,0,
    //                     0,0,0;
    // Biped->setGeneralizedCoordinate(jointNominalConfig);
    global_timer += 0.001;

    //joint angle position
    auto anglepos = robot.get_leg_pos();

    // dataFile << anglepos[0] << ", " << anglepos[1] << ", " <<anglepos[2] << ", "  
    //          << anglepos[3] << ", " << anglepos[4] << ", " <<anglepos[5] << ", " 
    //          << swc.postarget[0].x << ", " << swc.postarget[0].y << ", " <<swc.postarget[0].z << ", "  
    //          << swc.postarget[1].x << ", " << swc.postarget[1].y << ", " <<swc.postarget[1].z << ", "
    //          << stc.desired_states[0] << ", " << stc.desired_states[1] << ", " <<stc.desired_states[2] << ", "
    //          << stc.desired_states[3] << ", " << stc.desired_states[4] << ", " <<stc.desired_states[5] << ", "
    //          << stc.desired_states[6] << ", " << stc.desired_states[7] << ", " <<stc.desired_states[8] << ", "
    //          << stc.desired_states[9] << ", " << stc.desired_states[10] << ", " <<stc.desired_states[11] << ", "
    //          << stc.states[0] << ", " << stc.states[1] << ", " <<stc.states[2] << ", "
    //          << stc.states[3] << ", " << stc.states[4] << ", " <<stc.states[5] << ", "
    //          << stc.states[6] << ", " << stc.states[7] << ", " <<stc.states[8] << ", "
    //          << stc.states[9] << ", " << stc.states[10] << ", " <<stc.states[11] << ", "
    //          << RobotState.fe_r_pos_W[0] << ", "<< RobotState.fe_r_pos_W[1] << ", "<< RobotState.fe_r_pos_W[2] << ", " 
    //          << RobotState.fe_l_pos_W[0] << ", "<< RobotState.fe_l_pos_W[1] << ", "<< RobotState.fe_l_pos_W[2] << ", " 
    //          << RobotState.swing_fe_pos_des_W[0] << ", "<< RobotState.swing_fe_pos_des_W[1] << ", "<< RobotState.swing_fe_pos_des_W[2] << ", " 
    //          << RobotState.legState << ", " << wbc_time << ", "
    //          << RobotState.wbc_FrRes[0] << ", " << RobotState.wbc_FrRes[1] << ", " << RobotState.wbc_FrRes[2] << ", "
    //          << RobotState.wbc_FrRes[3] << ", " << RobotState.wbc_FrRes[4] << ", " << RobotState.wbc_FrRes[5] << ", "
    //          << RobotState.Fr_ff[0] << ", " << RobotState.Fr_ff[1] << ", " << RobotState.Fr_ff[2] << ", "
    //          << RobotState.Fr_ff[3] << ", " << RobotState.Fr_ff[4] << ", " << RobotState.Fr_ff[5] << ", "
    //          << std::endl;

    // dataFile << footplanner.footplacements_Xs[0] << ", " << footplanner.footplacements_Xs[1] << ", " << footplanner.footplacements_Xs[2] << ", " << footplanner.footplacements_Xs[3] << ", " << footplanner.footplacements_Xs[4] << ", "
    //          << footplanner.footplacements_Ys[0] << ", " << footplanner.footplacements_Ys[1] << ", " << footplanner.footplacements_Ys[2] << ", " << footplanner.footplacements_Ys[3] << ", " << footplanner.footplacements_Ys[4] << ", "
    //          << stc.states[3] << ", " << stc.states[4] << ", " <<stc.states[5] << ", "
    //          << footplanner.currentStancefootPosition_X << ", " << footplanner.currentStancefootPosition_Y << ", " << footplanner.currentStancefoot_ID
    //          << std::endl;
    std::cout<< swc.stance_foot_pos[1]<<std::endl;
    std::cout<< RobotState.stance_fe_pos_cur_W[1]<<std::endl;
    dataFile << swc.foothold_dcm[0] << ", " << swc.foothold_dcm[1] << ", " << swc.foothold_dcm[2] << ", "
             << swc.foothold_heuristic[0] << ", " << swc.foothold_heuristic[1] << ", " << swc.foothold_heuristic[2] << ", "
             << stc.states[3] << ", " << stc.states[4] << ", " <<stc.states[5] << ", "
             << footplanner.currentStancefootPosition_X << ", " << footplanner.currentStancefootPosition_Y << ", " << footplanner.currentStancefoot_ID << ", "
             << swc.foot_position_now[0][0] << ", " << swc.foot_position_now[0][1] << ", " << swc.foot_position_now[0][2] << ", "
             << swc.foot_position_now[1][0] << ", " << swc.foot_position_now[1][1] << ", " << swc.foot_position_now[1][2] << ", "
             << stc.states[9] << ", " << stc.states[10] << ", " 
             << RobotState.q(0)-swc.stance_foot_pos[0] << ", " << RobotState.q(1)-swc.stance_foot_pos[1] << ", " 
             << std::endl;

             
    server.integrateWorldThreadSafe();

  }

  dataFile.close();
  server.killServer();
}