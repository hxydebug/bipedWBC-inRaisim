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
 	swing_leg_controller swc(&robot,&gait_gen,0);
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
  Eigen::VectorXd user_cmd(4);
  float global_timer = 0;

  user_cmd<< 0.6,0.0,0.45,0.0;   //vx,vy,height,dyaw
  double x_com_desire=0.0;
  double y_com_desire=0.0;
  double yaw_desire=0.0;
  
  // calculate running time for wbc
  Timer t;

  while(1){
    raisim::MSLEEP(1);
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
    p_com_des<<0,0,0.5;//0.41~0.42
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
    leg_tau = l_control.get_action(2,user_cmd);
    l_control.dataBusWrite(RobotState);

    // ------------- WBC ------------
    // WBC input
    RobotState.des_ddq = Eigen::VectorXd::Zero(kinDynSolver.model_nv);
    RobotState.des_dq = Eigen::VectorXd::Zero(kinDynSolver.model_nv);
    RobotState.des_delta_q = Eigen::VectorXd::Zero(kinDynSolver.model_nv);
    x_com_desire +=  user_cmd[0] * 0.001;
    y_com_desire +=  user_cmd[1] * 0.001;
    yaw_desire += user_cmd[3] * 0.001;
    RobotState.base_rpy_des << 0, 0, yaw_desire;
    RobotState.base_pos_des << x_com_desire, y_com_desire, user_cmd[2];

    // adjust des_delata_q, des_dq and des_ddq to achieve forward walking
    if (global_timer>0.5) {
        RobotState.des_delta_q.block<2, 1>(0, 0) << user_cmd[0] *0.001, user_cmd[1] * 0.001;
        RobotState.des_delta_q(5) = user_cmd[3] * 0.001;
        RobotState.des_dq.block<2, 1>(0, 0) <<  user_cmd[0] ,  user_cmd[1] ;
        RobotState.des_dq(5) = user_cmd[3];

        double k = 1;
        RobotState.des_ddq.block<2, 1>(0, 0) << k * (user_cmd[0] - RobotState.dq(0)), k * (user_cmd[1] -
                                                                                              RobotState.dq(1));
        RobotState.des_ddq(5) = k * (user_cmd[3] - RobotState.dq(5));
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
      // cout<<RobotState.pBase_W<<endl;
      cout<<global_timer<<endl;
      // cout<<RobotState.q(0)<<" "<<RobotState.q(1)<<" "<<RobotState.q(2)<<endl;
      Eigen::VectorXd leg_tau1 = l_control.final_tau(RobotState);
      // cout<<leg_tau1<<endl;
      robot.step(leg_tau1,body_tau);
    }
    else{
      robot.step(leg_tau,body_tau);
    }
    float wbc_time = (float)t.getSeconds();
    // fix test 
    // Eigen::VectorXd jointNominalConfig(Biped->getGeneralizedCoordinateDim()), jointVelocityTarget(Biped->getDOF());
    // jointNominalConfig<<  0, 0, 0.6, RpyToqua(0.0*PII/180.0,0.0*PII/180.0,0.0*PII/180.0),
    //                     0,0,0,
    //                     0,0,0;
    // Biped->setGeneralizedCoordinate(jointNominalConfig);
    global_timer += 0.001;

    //joint angle position
    auto anglepos = robot.get_leg_pos();

    dataFile << anglepos[0] << ", " << anglepos[1] << ", " <<anglepos[2] << ", "  
             << anglepos[3] << ", " << anglepos[4] << ", " <<anglepos[5] << ", " 
             << swc.postarget[0].x << ", " << swc.postarget[0].y << ", " <<swc.postarget[0].z << ", "  
             << swc.postarget[1].x << ", " << swc.postarget[1].y << ", " <<swc.postarget[1].z << ", "
             << stc.desired_states[0] << ", " << stc.desired_states[1] << ", " <<stc.desired_states[2] << ", "
             << stc.desired_states[3] << ", " << stc.desired_states[4] << ", " <<stc.desired_states[5] << ", "
             << stc.desired_states[6] << ", " << stc.desired_states[7] << ", " <<stc.desired_states[8] << ", "
             << stc.desired_states[9] << ", " << stc.desired_states[10] << ", " <<stc.desired_states[11] << ", "
             << stc.states[0] << ", " << stc.states[1] << ", " <<stc.states[2] << ", "
             << stc.states[3] << ", " << stc.states[4] << ", " <<stc.states[5] << ", "
             << stc.states[6] << ", " << stc.states[7] << ", " <<stc.states[8] << ", "
             << stc.states[9] << ", " << stc.states[10] << ", " <<stc.states[11] << ", "
             << RobotState.fe_r_pos_W[0] << ", "<< RobotState.fe_r_pos_W[1] << ", "<< RobotState.fe_r_pos_W[2] << ", " 
             << RobotState.fe_l_pos_W[0] << ", "<< RobotState.fe_l_pos_W[1] << ", "<< RobotState.fe_l_pos_W[2] << ", " 
             << RobotState.swing_fe_pos_des_W[0] << ", "<< RobotState.swing_fe_pos_des_W[1] << ", "<< RobotState.swing_fe_pos_des_W[2] << ", " 
             << RobotState.legState << ", " << wbc_time << std::endl;

    server.integrateWorldThreadSafe();

  }

  dataFile.close();
  server.killServer();
}