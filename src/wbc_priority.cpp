/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
//
// Created by boxing on 23-12-29.
//
// Modified by Xinyan Huang on 24-11-13
// used for point foot biped robot
//

#include "wbc_priority.h"
#include "iostream"

int if_initialized = 0;
// QP_nvIn=12, QP_ncIn=16
WBC_priority::WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt) : QP_prob(QP_nvIn,
                                                                                                          QP_ncIn) {
    timeStep = dt;
    model_nv = model_nv_In;
    miu = miu_In;
    QP_nc = QP_ncIn;
    QP_nv = QP_nvIn;
    Sf = Eigen::MatrixXd::Zero(6, model_nv);
    Sf.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    St_qpV2 = Eigen::MatrixXd::Zero(model_nv, model_nv - 6); // 6 means the dims of floating base
    St_qpV2.block(6, 0, model_nv - 6, model_nv - 6) = Eigen::MatrixXd::Identity(model_nv - 6, model_nv - 6);

    St_qpV1 = Eigen::MatrixXd::Zero(model_nv, 6); // 6 means the dims of delta_b
    St_qpV1.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);

    // defined in body frame
    f_z_low = 0;
    f_z_upp = 1400;

    qpOASES::Options options;
    options.setToMPC();
    //options.setToReliable();
    options.printLevel = qpOASES::PL_NONE;
    QP_prob.setOptions(options);

    eigen_xOpt = Eigen::VectorXd::Zero(QP_nv);
    eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
    eigen_fr_Opt = Eigen::VectorXd::Zero(6);
    eigen_tau_Opt = Eigen::VectorXd::Zero(model_nv - 6);

    delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
    dq_final_kin = Eigen::VectorXd::Zero(model_nv);;
    ddq_final_kin = Eigen::VectorXd::Zero(model_nv);

    base_rpy_cur = Eigen::VectorXd::Zero(3);

    //  WBC task defined and order build
    ///------------ walk --------------
    kin_tasks_walk.addTask("static_Contact");
    kin_tasks_walk.addTask("SwingLeg");
    kin_tasks_walk.addTask("PosRot");

    std::vector<std::string> taskOrder_walk;
    taskOrder_walk.emplace_back("static_Contact");
    // taskOrder_walk.emplace_back("SwingLeg");
    taskOrder_walk.emplace_back("PosRot");
    taskOrder_walk.emplace_back("SwingLeg");

    kin_tasks_walk.buildPriority(taskOrder_walk);

    ///-------- stand ------------
    // kin_tasks_stand.addTask("static_Contact");
    // kin_tasks_stand.addTask("Pz");
    // kin_tasks_stand.addTask("CoMXY_HipRPY");

    // std::vector<std::string> taskOrder_stand;

    // taskOrder_stand.emplace_back("static_Contact");
    // taskOrder_stand.emplace_back("CoMXY_HipRPY");
    // taskOrder_stand.emplace_back("Pz");

    // kin_tasks_stand.buildPriority(taskOrder_stand);
}

void WBC_priority::dataBusRead(const DataBus &robotState) {
    // foot-end offset posture
    fe_L_rot_L_off = robotState.fe_L_rot_L_off;
    fe_R_rot_L_off = robotState.fe_R_rot_L_off;

    // deisred values
    base_rpy_des = robotState.base_rpy_des;
    base_rpy_cur << robotState.rpy[0], robotState.rpy[1], robotState.rpy[2];
    base_pos_des = robotState.base_pos_des;
    swing_fe_pos_des_W = robotState.swing_fe_pos_des_W;
    swing_fe_rpy_des_W = robotState.swing_fe_rpy_des_W;
    stance_fe_pos_cur_W = robotState.stance_fe_pos_cur_W;
    stance_fe_rot_cur_W = robotState.stance_fe_rot_cur_W;
    stanceDesPos_W = robotState.stanceDesPos_W;
    fe_l_pos_cur_W = robotState.fe_l_pos_W;
    fe_r_pos_cur_W = robotState.fe_r_pos_W;
    fe_l_rot_cur_W = robotState.fe_l_rot_W;
    fe_r_rot_cur_W = robotState.fe_r_rot_W;
    des_ddq = robotState.des_ddq;
    des_dq = robotState.des_dq;
    des_delta_q = robotState.des_delta_q;
    des_q = robotState.des_q;

    // state update
    J_base = robotState.J_base;
    dJ_base = robotState.dJ_base;
    base_rot = robotState.base_rot;
    base_pos = robotState.base_pos;

    Jfe = Eigen::MatrixXd::Zero(6, model_nv);
    Jfe.block(0, 0, 3, model_nv) = robotState.J_l.block(0, 0, 3, model_nv);;
    Jfe.block(3, 0, 3, model_nv) = robotState.J_r.block(0, 0, 3, model_nv);;
    dJfe = Eigen::MatrixXd::Zero(6, model_nv);
    dJfe.block(0, 0, 3, model_nv) = robotState.dJ_l.block(0, 0, 3, model_nv);;
    dJfe.block(3, 0, 3, model_nv) = robotState.dJ_r.block(0, 0, 3, model_nv);;
    Fr_ff = robotState.Fr_ff;
    dyn_M = robotState.dyn_M;
    dyn_M_inv = robotState.dyn_M_inv;
    dyn_Ag = robotState.dyn_Ag;
    dyn_dAg = robotState.dyn_dAg;
    dyn_Non = robotState.dyn_Non;
    dq = robotState.dq;
    q = robotState.q;
    legStateCur = robotState.legState;
    motionStateCur = robotState.motionState;

    if (legStateCur == DataBus::LSt) {
        Jc = robotState.J_l;
        dJc = robotState.dJ_l;
        Jsw = robotState.J_r;
        dJsw = robotState.dJ_r;
        fe_pos_sw_W = robotState.fe_r_pos_W;
        fe_rot_sw_W = robotState.fe_r_rot_W;
    } else {
        Jc = robotState.J_r;
        dJc = robotState.dJ_r;
        Jsw = robotState.J_l;
        dJsw = robotState.dJ_l;
        fe_pos_sw_W = robotState.fe_l_pos_W;
        fe_rot_sw_W = robotState.fe_l_rot_W;
    }

    Jcom=robotState.Jcom_W;
    pCoMCur=robotState.pCoM_W;

}

void WBC_priority::dataBusWrite(DataBus &robotState) {
    robotState.wbc_ddq_final = eigen_ddq_Opt;
    robotState.wbc_tauJointRes = tauJointRes;
    robotState.wbc_FrRes = eigen_fr_Opt;
    robotState.qp_cpuTime = cpu_time;
    robotState.qp_nWSR = nWSR;
    robotState.qp_status = qpStatus;

    robotState.wbc_delta_q_final = delta_q_final_kin;
    robotState.wbc_dq_final = dq_final_kin;
    robotState.wbc_ddq_final = ddq_final_kin;

    robotState.qp_status = qpStatus;
    robotState.qp_nWSR = nWSR;
    robotState.qp_cpuTime = cpu_time;
}

// QP problem contains joint torque, QP_nv=6+6, QP_nc=16;
void WBC_priority::computeTau() {
    // constust the QP problem, refer to the md file for more details
    Eigen::MatrixXd eigen_qp_A1 = Eigen::MatrixXd::Zero(6, QP_nv);// 6+6 means the sum of dims of delta_r and delta_Fr
    eigen_qp_A1.block<6, 6>(0, 0) = Sf * dyn_M * St_qpV1;

    eigen_qp_A1.block<6, 6>(0, 6) = -Sf * Jfe.transpose();
    // std::cout<< eigen_qp_A1.rows()<< "x" << eigen_qp_A1.cols() << std::endl;
    // std::cout<<"Sf"<< Sf.rows()<< "x" << Sf.cols() << std::endl;
    // std::cout<< dyn_M.rows()<< "x" << dyn_M.cols() << std::endl;
    // std::cout<< ddq_final_kin.rows()<< "x" << ddq_final_kin.cols() << std::endl;
    // std::cout<<"dyn_Non"<< dyn_Non.rows()<< "x" << dyn_Non.cols() << std::endl;
    // std::cout<< Jfe.rows()<< "x" << Jfe.cols() << std::endl;
    // std::cout<< Fr_ff.rows()<< "x" << Fr_ff.cols() << std::endl;
    Eigen::VectorXd eqRes = Eigen::VectorXd::Zero(6);
    eqRes = -Sf * dyn_M * ddq_final_kin - Sf * dyn_Non + Sf * Jfe.transpose() * Fr_ff;
    // std::cout<< eqRes.rows()<< "x" << eqRes.cols() << std::endl;
    // only the plane foot or line foot need multiply the rot matrix
    // Eigen::Matrix3d Rfe;
    // Rfe = stance_fe_rot_cur_W;

    // Eigen::Matrix<double,6,6> Mw2b;
    // Mw2b.setZero();
    // Mw2b.block(0,0,3,3)=Rfe.transpose();
    // Mw2b.block(3,3,3,3)=Rfe.transpose();

    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(10, 6);
    // std::cout<< W << std::endl;
    // std::cout<< W.rows()<< "x" << W.cols() << std::endl;
    W(0, 0) = 1;
    W(0, 2) = miu;
    W(1, 0) = -1;
    W(1, 2) = miu;
    W(2, 1) = 1;
    W(2, 2) = miu;
    W(3, 1) = -1;
    W(3, 2) = miu;
    W(4, 2) = 1;
    W.block<5, 3>(5, 3) = W.block<5, 3>(0, 0);
    // W=W*Mw2b;
//  std::cout<< W.rows()<< "x" << W.cols() << std::endl;
    Eigen::VectorXd f_low = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd f_upp = Eigen::VectorXd::Zero(10);
//    std::cout<<"wbc_computeTau, st_fe_rot"<<std::endl<<stance_fe_rot_cur_W<<std::endl;

    f_upp.block<5, 1>(0, 0) << 1e10, 1e10, 1e10, 1e10, f_z_upp;
    f_upp.block<5, 1>(5, 0) = f_upp.block<5, 1>(0, 0);
    f_low.block<5, 1>(0, 0) << 0, 0, 0, 0, f_z_low;
    f_low.block<5, 1>(5, 0) = f_low.block<5, 1>(0, 0);

    if (motionStateCur == DataBus::Walk  || motionStateCur==DataBus::Walk2Stand) {
        if (legStateCur == DataBus::LSt) {
            f_upp(9) = 0;

            f_low(9) = 0;

            f_low(5) = -1e-7;
            f_low(6) = -1e-7;
            f_low(7) = -1e-7;
            f_low(8) = -1e-7;
        } else if (legStateCur == DataBus::RSt) {
            f_upp(4) = 0;

            f_low(4) = 0;  

            f_low(0) = -1e-7;
            f_low(1) = -1e-7;
            f_low(2) = -1e-7;
            f_low(3) = -1e-7;
        }
    }
    // std::cout<<"11"<<std::endl;
    Eigen::MatrixXd eigen_qp_A2 = Eigen::MatrixXd::Zero(10, 12);
    eigen_qp_A2.block<10, 6>(0, 6) = W;
    Eigen::VectorXd neqRes_low = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd neqRes_upp = Eigen::VectorXd::Zero(10);

    neqRes_low = f_low - W * Fr_ff;
    neqRes_upp = f_upp - W * Fr_ff;

    Eigen::MatrixXd eigen_qp_A_final = Eigen::MatrixXd::Zero(QP_nc, QP_nv);
    eigen_qp_A_final.block<6, 12>(0, 0) = eigen_qp_A1;
    eigen_qp_A_final.block<10, 12>(6, 0) = eigen_qp_A2;

    Eigen::VectorXd eigen_qp_lbA = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd eigen_qp_ubA = Eigen::VectorXd::Zero(16);

    eigen_qp_lbA.block<6, 1>(0, 0) = eqRes;
    eigen_qp_lbA.block<10, 1>(6, 0) = neqRes_low;
    eigen_qp_ubA.block<6, 1>(0, 0) = eqRes;
    eigen_qp_ubA.block<10, 1>(6, 0) = neqRes_upp;

    Eigen::MatrixXd eigen_qp_H = Eigen::MatrixXd::Zero(QP_nv, QP_nv);
    Q2 = Eigen::MatrixXd::Identity(6, 6);
    Q1 = Eigen::MatrixXd::Identity(6, 6);
    eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;// more dynamics
    eigen_qp_H.block<6, 6>(6, 6) = Q1 * 2.0 * 1e7;// more mpc

    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //       lb<=x<=ub
//    qpOASES::real_t qp_H[QP_nv*QP_nv];
//    qpOASES::real_t qp_A[QP_nc*QP_nv];
//    qpOASES::real_t qp_g[QP_nv];
//    qpOASES::real_t qp_lbA[QP_nc];
//    qpOASES::real_t qp_ubA[QP_nc];
//    qpOASES::real_t xOpt_iniGuess[QP_nv];

    copy_Eigen_to_real_t(qp_H, eigen_qp_H, eigen_qp_H.rows(), eigen_qp_H.cols());
    copy_Eigen_to_real_t(qp_A, eigen_qp_A_final, eigen_qp_A_final.rows(), eigen_qp_A_final.cols());
    copy_Eigen_to_real_t(qp_lbA, eigen_qp_lbA, eigen_qp_lbA.rows(), eigen_qp_lbA.cols());
    copy_Eigen_to_real_t(qp_ubA, eigen_qp_ubA, eigen_qp_ubA.rows(), eigen_qp_ubA.cols());

    qpOASES::returnValue res;
    for (int i = 0; i < QP_nv; i++) {
        xOpt_iniGuess[i] = 0;
//        xOpt_iniGuess[i] =eigen_xOpt(i);
        qp_g[i] = 0;
    }
    if(if_initialized==0){
        nWSR = 200;
        cpu_time = timeStep;
    //    QP_prob.reset();
        res = QP_prob.init(qp_H, qp_g, qp_A, NULL, NULL, qp_lbA, qp_ubA, nWSR, &cpu_time, xOpt_iniGuess);
        // if_initialized = 1;
    }
    else{
        nWSR = 200;
        cpu_time = timeStep;
    //    QP_prob.reset();
        res = QP_prob.hotstart(qp_g, NULL, NULL, qp_lbA, qp_ubA, nWSR, NULL);
    }
    
    qpStatus = qpOASES::getSimpleStatus(res);
//    if (res==qpOASES::SUCCESSFUL_RETURN)
//        printf("WBC-QP: successful_return\n");
//    else if (res==qpOASES::RET_MAX_NWSR_REACHED)
//        printf("WBC-QP: max_nwsr\n");
//    else if (res==qpOASES::RET_INIT_FAILED)
//        printf("WBC-QP: init_failed\n");

    qpOASES::real_t xOpt[QP_nv];
    QP_prob.getPrimalSolution(xOpt);
    if (res == qpOASES::SUCCESSFUL_RETURN)
        for (int i = 0; i < QP_nv; i++)
            eigen_xOpt(i) = xOpt[i];

    eigen_ddq_Opt = ddq_final_kin;
    eigen_ddq_Opt.block<6, 1>(0, 0) += eigen_xOpt.block<6, 1>(0, 0);
    eigen_fr_Opt = Fr_ff + eigen_xOpt.block<6, 1>(6, 0);
    // std::cout<<"eigen_fr_Opt: "<<eigen_fr_Opt.transpose()<<std::endl;
    // std::cout<<"Fr_ff: "<<Fr_ff.transpose()<<std::endl;

    if (qpStatus != 0){
        Eigen::MatrixXd A_x;
        Eigen::VectorXd xOpt_iniGuess_m(QP_nv,1);
        for (int i=0; i < QP_nv;i++)
            xOpt_iniGuess_m(i) = xOpt_iniGuess[i];

    }

    Eigen::VectorXd tauRes;
    tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe.transpose() * eigen_fr_Opt;

    tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);
//    std::cout<<"qpRes_frOpt"<<std::endl;
//    std::cout<<eigen_fr_Opt.transpose()<<std::endl;

    last_nWSR = nWSR;
    last_cpu_time = cpu_time;
}

void WBC_priority::computeDdq(Pin_KinDyn &pinKinDynIn) {
    // task definition
    /// -------- walk -------------
    {
        int id = kin_tasks_walk.getId("static_Contact");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 0;
        kin_tasks_walk.taskLib[id].J = Jc.block(0, 0, 3, model_nv);
//            kin_tasks_walk.taskLib[id].J.block(0,22,3,3).setZero();
        kin_tasks_walk.taskLib[id].dJ = dJc.block(0, 0, 3, model_nv);
//            kin_tasks_walk.taskLib[id].dJ.block(0,22,3,3).setZero();
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("PosRot");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].errX.block(0,0,3,1) = base_pos_des - q.block(0,0,3,1);
        // std::cout<< "desired height: "<<base_pos_des[2]<<std::endl;
        // std::cout<< "current height: "<<q[2]<<std::endl;
        if (fabs(kin_tasks_walk.taskLib[id].errX(0))>=0.02)
            kin_tasks_walk.taskLib[id].errX(0)=0.02* sign(kin_tasks_walk.taskLib[id].errX(0));
        if (fabs(kin_tasks_walk.taskLib[id].errX(1))>=0.01)
            kin_tasks_walk.taskLib[id].errX(1)=0.01* sign(kin_tasks_walk.taskLib[id].errX(1));
        // if (kin_tasks_walk.taskLib[id].errX(2)>0.005)
        //     kin_tasks_walk.taskLib[id].errX(2)=0.005;
        Eigen::Matrix3d desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(base_rot, desRot);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].derrX = des_dq.block(0,0,6,1) - dq.block(0,0,6,1);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * 500;
        kin_tasks_walk.taskLib[id].kp.block(0,0,2,2)=Eigen::MatrixXd::Identity(2, 2) * 0.0;
        kin_tasks_walk.taskLib[id].kp.block<1, 1>(2, 2)=Eigen::MatrixXd::Identity(1, 1) * 800;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * 10;
        kin_tasks_walk.taskLib[id].kd.block(0,0,2,2)=Eigen::MatrixXd::Identity(2, 2) * 0.0;
        // kin_tasks_walk.taskLib[id].kd.block<1, 1>(2, 2)=Eigen::MatrixXd::Identity(1, 1) * 100;
        kin_tasks_walk.taskLib[id].J = J_base;
        kin_tasks_walk.taskLib[id].dJ = dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("SwingLeg");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(0, 0) = swing_fe_pos_des_W - fe_pos_sw_W;
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
//        kin_tasks_walk.taskLib[id].derrX=-Jsw*dq;
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 2000;
        kin_tasks_walk.taskLib[id].kp.block<1, 1>(2, 2) = kin_tasks_walk.taskLib[id].kp.block<1, 1>(2, 2) * 0.1;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 20;
        kin_tasks_walk.taskLib[id].J = Jsw.block(0, 0, 3, model_nv);
        kin_tasks_walk.taskLib[id].dJ = dJsw.block(0, 0, 3, model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    }

    /// -------- stand -------------
//     {

//         int id = kin_tasks_stand.getId("static_Contact");
//         kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(6);
//         kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
//         kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
//         kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
//         kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * 0;
//         kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * 0;
//         kin_tasks_stand.taskLib[id].J=Eigen::MatrixXd::Zero(6,model_nv);
//         kin_tasks_stand.taskLib[id].J=Jfe;
//         kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(6,model_nv);
//         kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

//         id = kin_tasks_stand.getId("Pz");
//         kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(1);
//         kin_tasks_stand.taskLib[id].errX(0) = base_pos_des(2) - q(2);
//         kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(1);
//         kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(1);
//         kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(1);
//         kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(1, 1) * 2000; //100
//         kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(1, 1) * 10;
//         Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(1, 6);
//         taskMap(0, 2) = 1;
//         kin_tasks_stand.taskLib[id].J = taskMap * J_base;
//         kin_tasks_stand.taskLib[id].dJ = taskMap * dJ_base;
//         kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

//         id = kin_tasks_stand.getId("CoMXY_HipRPY");
//         Eigen::MatrixXd taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
//         taskMapRPY(0, 3) = 1;
//         taskMapRPY(1, 4) = 1;
//         taskMapRPY(2, 5) = 1;
//         kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(5);
//         kin_tasks_stand.taskLib[id].errX.block(0,0,2,1) = pCoMDes.block(0,0,2,1)-pCoMCur.block(0,0,2,1);
// //            kin_tasks_stand.taskLib[id].errX[0]+=0.01;
//         Eigen::Matrix3d desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
//         kin_tasks_stand.taskLib[id].errX.block<3, 1>(2, 0) = diffRot(hip_link_rot, desRot);
//         kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(5);
// //            kin_tasks_stand.taskLib[id].derrX.block(0,0,2,1)=-(Jcom*dq).block(0,0,2,1);
// //            kin_tasks_stand.taskLib[id].derrX.block(2,0,3,1)=-taskMapRPY*J_hip_link*dq;
//         kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(5);
//         kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(5);
//         kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(5, 5) * 1000; //100
//         kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(5, 5) * 10;
//         kin_tasks_stand.taskLib[id].kp.block(2,2,3,3) = Eigen::MatrixXd::Identity(3, 3) * 1000; //100 // for hip rpy
//         kin_tasks_stand.taskLib[id].kd.block(2,2,3,3) = Eigen::MatrixXd::Identity(3, 3) * 10; //100  // for hip rpy
//         kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(5,model_nv);
//         kin_tasks_stand.taskLib[id].J.block(0,0,2,model_nv) = Jcom.block(0,0,2,model_nv);
//         kin_tasks_stand.taskLib[id].J.block(2,0,3,model_nv) = taskMapRPY * J_hip_link;
//         //kin_tasks_stand.taskLib[id].J.block(0,6,2,14).setZero();
//         kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(5,model_nv);
//         kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

//     }

    if (motionStateCur==DataBus::Walk || motionStateCur==DataBus::Walk2Stand) {
        kin_tasks_walk.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_walk.out_delta_q;
        dq_final_kin = kin_tasks_walk.out_dq;
        ddq_final_kin = kin_tasks_walk.out_ddq;
    }
    // else if (motionStateCur==DataBus::Stand) {
    //     kin_tasks_stand.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
    //     delta_q_final_kin = kin_tasks_stand.out_delta_q;
    //     dq_final_kin = kin_tasks_stand.out_dq;
    //     ddq_final_kin = kin_tasks_stand.out_ddq;
    else
    {
        delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
        dq_final_kin = Eigen::VectorXd::Zero(model_nv);
        ddq_final_kin = Eigen::VectorXd::Zero(model_nv);
    }

    // final WBC output collection



}

void WBC_priority::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols) {
    int count = 0;

    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count++] = std::isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}

void WBC_priority::setQini(const Eigen::VectorXd &qIniDesIn, const Eigen::VectorXd &qIniCurIn) {
    qIniDes = qIniDesIn;
    qIniCur = qIniCurIn;
}