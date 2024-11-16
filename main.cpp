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

  user_cmd<< -0.2,0.0,0.45,0;   //vx,vy,height,dyaw
  
  while(1){
    raisim::MSLEEP(1);
    // get sensor data
    robot.update_state();
    robot.dataBusWrite(RobotState);

    // update kinematics and dynamics info
    kinDynSolver.dataBusRead(RobotState);
    kinDynSolver.computeJ_dJ();
    kinDynSolver.computeDyn();
    kinDynSolver.dataBusWrite(RobotState);

    Eigen::Vector3d p_com_des,w_com_des,dp_com_des,dw_com_des;
    p_com_des<<0,0,0.5;//0.41~0.42
    dp_com_des<<0,0,0;
    w_com_des<<0,0,0;
    dw_com_des<<0,0,0;
    body_tau = l_control.control_body_directly(p_com_des, w_com_des, dp_com_des, dw_com_des);
    if(global_timer>0.5)  
    body_tau << 0,0,0,0,0,0;
    // if(global_timer>2)  body_tau = l_control.control_body_directly2(p_com_des, w_com_des, dp_com_des, dw_com_des);
    // if(global_timer>5)  
    // body_tau << 0,0,0,0,0,0;
    leg_tau = l_control.get_action(2,user_cmd);
    // leg_tau << 0,0,0,0,0,0;
    // cout << leg_tau <<endl;
    robot.step(leg_tau,body_tau);
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
    //          << stc.states[9] << ", " << stc.states[10] << ", " <<stc.states[11] << std::endl;

    server.integrateWorldThreadSafe();

  }

  dataFile.close();
  server.killServer();
}