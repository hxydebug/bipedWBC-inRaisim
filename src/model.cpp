#include "model.h"
#include "leg_controller.h"

using namespace std;

#define PII 3.1415926535

robot::robot(raisim::ArticulatedSystem* client){
	biped = client;
    robot::initial_robot(); 
    robot::update_state();   
}
void robot::initial_robot(){
    float hight = 0.38;
    Angle angle_l,angle_r;
    set_xyz(fl,&angle_l,0.05,0.08,-0.32);
    set_xyz(fr,&angle_r,0.05,-0.08,-0.32);
    Eigen::VectorXd jointNominalConfig(biped->getGeneralizedCoordinateDim()), jointVelocityTarget(biped->getDOF());
    jointNominalConfig<< 0, 0, hight, RpyToqua(0.0*PII/180.0,0.0*PII/180.0,0.0*PII/180.0),  
                         angle_l.q[0],angle_l.q[1],angle_l.q[2],
                         angle_r.q[0],angle_r.q[1],angle_r.q[2];

    // // fixed base
    // jointNominalConfig<<  0,0,0,
    //                     0,0,0;

    jointVelocityTarget.setZero();
    jointVelocityTarget[0] = 0.0;//velocity
    biped->setGeneralizedCoordinate(jointNominalConfig);
    biped->setGeneralizedVelocity(jointVelocityTarget);
    biped->setGeneralizedForce(Eigen::VectorXd::Zero(biped->getDOF()));
    biped->setName("smart biped");
}
int robot::getDOF(){
    return biped->getDOF();
}
int robot::getGeneralizedCoordinateDim(){
    return biped->getGeneralizedCoordinateDim();
}
void robot::step(Eigen::VectorXd leg_tau,Eigen::VectorXd body_tau){
    Eigen::VectorXd tau(12);
    tau<<body_tau,leg_tau;
    // tau<<leg_tau;
    // cout << tau << endl;
    biped->setGeneralizedForce(tau);
}
void robot::update_state(){
    biped->getState(pos,vel);

    // cout<< "pos" << endl;
    // cout<< pos << endl;

    // cout<< "vel" << endl;
    // cout << vel <<endl;
    rot_matrix = biped->getBaseOrientation().e();
    robot::update_angle();
}
void robot::update_angle(){
    Eigen::Vector3d angle_v = robot::get_base_angle_v();
    dpsi = (angle_v[2]+last_dpsi)/2;
    dvarphi = (angle_v[0]+last_dvarphi)/2;
    last_dpsi = angle_v[2];
    last_dvarphi = angle_v[0];
    // dpsi = angle_v[2];
    // dvarphi = angle_v[0];
}
// void robot::update_angle(){
//     Eigen::Vector3d angle_v = robot::get_base_angle_v();
//     dpsi = (angle_v[2]+last_dpsi+llast_dpsi)/3;
//     dvarphi = (angle_v[0]+last_dvarphi+llast_dvarphi)/3;
//     llast_dpsi = last_dpsi;
//     llast_dvarphi = last_dvarphi;
//     last_dpsi = angle_v[2];
//     last_dvarphi = angle_v[0];
// }

Eigen::VectorXd robot::get_p_com(){
    return pos.head(3);
}
Eigen::Matrix3d robot::get_com_rotmatrix(){
    return rot_matrix;
}
Eigen::VectorXd robot::get_dp_com(){
    return vel.head(3);
}
Eigen::VectorXd robot::get_dw_com(){
    return vel.segment(3,3);
}

Eigen::VectorXd robot::get_leg_vel(){
    return vel.tail(6);
}

Eigen::VectorXd robot::get_leg_pos(){
    return pos.tail(6);
}

Eigen::VectorXd robot::get_base_orientation(){
    return pos.segment(3,4);
}
Eigen::VectorXd robot::get_base_rpy(){
    Eigen::Vector3d rpy;
    Eigen::Vector4d quat = pos.segment(3,4);
    quaToRpy(quat,rpy);
    return rpy;
}
Eigen::VectorXd robot::get_base_angle_v(){
    Eigen::Vector3d angle_v;
    Eigen::Vector3d angle_v_world = vel.segment(3,3);
    angle_v = rot_matrix.transpose()*angle_v_world;
    return angle_v;
}
double robot::get_psi(){
    Eigen::Vector3d rpy = robot::get_base_rpy();
    return rpy[2];
}
double robot::get_varphi(){
    Eigen::Vector3d rpy = robot::get_base_rpy();
    return rpy[0];
}

double robot::get_dpsi(){
    return dpsi;
}
double robot::get_dvarphi(){
    return dvarphi;
}
double robot::get_base_v(){
    Eigen::Vector3d v;
    Eigen::Vector3d v_world = vel.head(3);
    v = rot_matrix.transpose()*v_world;
    return v[0];
}
Eigen::Vector3d robot::get_base_vec(){
    Eigen::Vector3d v_world = vel.head(3);
    return rot_matrix.transpose()*v_world;
}

std::vector<int> robot::GetFootContact(){
    auto l_foot = biped->getBodyIdx("FL_foot");
    auto r_foot = biped->getBodyIdx("FR_foot");
    contact_leg[0] = 0;
    contact_leg[1] = 0;
    for(auto& contact: biped->getContacts()) {
        if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
        if ( l_foot == contact.getlocalBodyIndex() ) {
            contact_leg[0] = 1;
        }
        if ( r_foot == contact.getlocalBodyIndex() ) {
            contact_leg[1] = 1;
        }
    }
    return contact_leg;
}