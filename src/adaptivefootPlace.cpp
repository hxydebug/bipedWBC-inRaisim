#include "adaptivefootPlace.h"


using qpOASES::QProblem;
using namespace std;

void copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols);


FootHoldPlanner::FootHoldPlanner(double comHeight, double stepPeriod, double averageSpeed, double stepWidth)
{
    g = 9.8;
    h = comHeight;
    omega = sqrt(g/h);
    stepDuration = stepPeriod;
    averageSpeedX = averageSpeed;
    stepLengthSteady = averageSpeedX * stepDuration;
    stepWidthSteady = stepWidth;

    // calculate dcm offset
    dcmXSteady = stepLengthSteady/(FootHoldPlanner::deltaTransformation(stepDuration) -1);
    dcmYSteady = stepWidthSteady/(FootHoldPlanner::deltaTransformation(stepDuration) +1);

}

Eigen::VectorXd FootHoldPlanner::ComputeNextfootHold(int Nsteps,
                                        double dcmOffsetX,
                                        double dcmOffsetY,
                                        int currentStancefootID,
                                        double currentStancefootPositionX,
                                        double currentStancefootPositionY,
                                        double phase)
{
    leftoverTime = stepDuration*(1.0-phase);
    FootHoldPlanner::getStanceFootSequence(Nsteps,currentStancefootID);
    // calculate the same A_qp and B_qp
    Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(1, 1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, 1);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(1, 1);
    A0(0,0) = FootHoldPlanner::deltaTransformation(leftoverTime);
    A(0,0) = FootHoldPlanner::deltaTransformation(stepDuration);
    B(0,0) = -1.0;
    A_qp = Eigen::MatrixXd::Zero(Nsteps, 1);
    Eigen::MatrixXd B_aux = Eigen::MatrixXd::Zero(Nsteps, 1);
    B_qp = Eigen::MatrixXd::Zero(Nsteps, Nsteps);
    A_qp.block(0, 0, 1, 1) = A0;
    for (int i = 1; i < Nsteps; ++i) {
        A_qp.block<1, 1>(i * 1, 0) =
            A * A_qp.block<1, 1>((i - 1) * 1, 0);
    }
    // compute [B, A * B, ..., A^(N-1) * B]
    B_aux.block(0, 0, 1, 1) = B;
    for (int i = 1; i < Nsteps; ++i) {
        B_aux.block(i * 1, 0, 1, 1) =
            A * B_aux.block((i - 1) * 1, 0, 1, 1);
    }
    for (int i = 0; i < Nsteps; ++i) {
        // Diagonal block.
        B_qp.block(i * 1, i * 1, 1, 1) = B;
        // Off diagonal Diagonal block = A^(i - j - 1) * B.
        for (int j = 0; j < i; ++j) {
            const int power = i - j;
            B_qp.block(i * 1, j * 1, 1, 1) =
                B_aux.block(power * 1, 0, 1, 1);
        }
    }
    Eigen::VectorXd footplacements_X = FootHoldPlanner::optimalLongitudinalFootPlacement(Nsteps, dcmOffsetX);
    Eigen::VectorXd footplacements_Y = FootHoldPlanner::optimalLateralFootPlacement(Nsteps, dcmOffsetY);
    Eigen::VectorXd footHold = Eigen::VectorXd::Zero(2);
    footHold << footplacements_X[0], footplacements_Y[0];
    return footHold;
}

Eigen::VectorXd FootHoldPlanner::optimalLongitudinalFootPlacement(int Nsteps, double dcmOffsetX){
    
    double x0 = dcmOffsetX;
    double x_low = -1.0;
    double x_high = 1.0;
    double u_low = -0.9;
    double u_high = 0.9;
    Eigen::VectorXd x_low_Vector = Eigen::VectorXd::Zero(Nsteps);
    Eigen::VectorXd x_high_Vector = Eigen::VectorXd::Zero(Nsteps);
    Eigen::VectorXd u_low_Vector = Eigen::VectorXd::Zero(Nsteps);
    Eigen::VectorXd u_high_Vector = Eigen::VectorXd::Zero(Nsteps);
    x_low_Vector.setConstant(x_low);
    x_high_Vector.setConstant(x_high);
    u_low_Vector.setConstant(u_low);
    u_high_Vector.setConstant(u_high);
    Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(Nsteps);
    x_ref.setConstant(dcmXSteady);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Nsteps, Nsteps) * 1.0 * 1e3;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Nsteps, Nsteps) * 1.0 * 1e3;
    Eigen::MatrixXd eigen_qp_H = R + B_qp.transpose() * Q * B_qp;
    Eigen::MatrixXd eigen_qp_A = B_qp;
    Eigen::MatrixXd eigen_qp_g = B_qp.transpose() * Q * (A_qp * x0 - x_ref);
    Eigen::MatrixXd eigen_qp_lbA = x_low_Vector - A_qp * x0;
    Eigen::MatrixXd eigen_qp_ubA = x_high_Vector - A_qp * x0;
    Eigen::MatrixXd eigen_qp_lb = u_low_Vector;
    Eigen::MatrixXd eigen_qp_ub = u_high_Vector;
    
    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //       lb<=x<=ub
    qpOASES::real_t qp_H[Nsteps*Nsteps];
    qpOASES::real_t qp_A[Nsteps*Nsteps];
    qpOASES::real_t qp_g[Nsteps];
    qpOASES::real_t qp_lbA[Nsteps];
    qpOASES::real_t qp_ubA[Nsteps];
    qpOASES::real_t qp_lb[Nsteps];
    qpOASES::real_t qp_ub[Nsteps];

    copy_Eigen_to_real_t(qp_H, eigen_qp_H, eigen_qp_H.rows(), eigen_qp_H.cols());
    copy_Eigen_to_real_t(qp_A, eigen_qp_A, eigen_qp_A.rows(), eigen_qp_A.cols());
    copy_Eigen_to_real_t(qp_g, eigen_qp_g, eigen_qp_g.rows(), eigen_qp_g.cols());
    copy_Eigen_to_real_t(qp_lbA, eigen_qp_lbA, eigen_qp_lbA.rows(), eigen_qp_lbA.cols());
    copy_Eigen_to_real_t(qp_ubA, eigen_qp_ubA, eigen_qp_ubA.rows(), eigen_qp_ubA.cols());
    copy_Eigen_to_real_t(qp_lb, eigen_qp_lb, eigen_qp_lb.rows(), eigen_qp_lb.cols());
    copy_Eigen_to_real_t(qp_ub, eigen_qp_ub, eigen_qp_ub.rows(), eigen_qp_ub.cols());    

    auto qp_problem = QProblem(Nsteps, Nsteps, qpOASES::HST_UNKNOWN,
                               qpOASES::BT_TRUE);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp_problem.setOptions(options);

    int max_solver_iter = 100;

    qp_problem.init(qp_H, qp_g, qp_A, qp_lb,
                    qp_ub, qp_lbA, qp_ubA, max_solver_iter,
                    nullptr);

    std::vector<qpOASES::real_t> qp_sol(Nsteps, 0);
    qp_problem.getPrimalSolution(qp_sol.data());
    Eigen::VectorXd footplacements_X = Eigen::VectorXd::Zero(Nsteps);
    for (int i = 0; i < Nsteps; ++i) {
        footplacements_X[i] = qp_sol[i];
    }
    return footplacements_X;
}

Eigen::VectorXd FootHoldPlanner::optimalLateralFootPlacement(int Nsteps, double dcmOffsetY){

    double x0 = dcmOffsetY;
    double x_low_left = -1;
    double x_high_left = 0;
    double x_low_right = 0;
    double x_high_right = 1;

    double u_low_left = 0.0;
    double u_high_left = 0.9;
    double u_low_right = -0.9;
    double u_high_right = -0.0;
    Eigen::VectorXd x_low_Vector = Eigen::VectorXd::Zero(Nsteps);
    Eigen::VectorXd x_high_Vector = Eigen::VectorXd::Zero(Nsteps);
    Eigen::VectorXd u_low_Vector = Eigen::VectorXd::Zero(Nsteps);
    Eigen::VectorXd u_high_Vector = Eigen::VectorXd::Zero(Nsteps);
    Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(Nsteps);
    for (int i = 0; i < Nsteps; ++i) {
        if(stanceFootSeq[i]==0){
            // left foot touch down
            x_low_Vector[i] = x_low_left;
            x_high_Vector[i] = x_high_left;
            u_low_Vector[i] = u_low_left;
            u_high_Vector[i] = u_high_left;
            x_ref[i] = -dcmYSteady;
        }
        else{
            // right foot touch down
            x_low_Vector[i] = x_low_right;
            x_high_Vector[i] = x_high_right;
            u_low_Vector[i] = u_low_right;
            u_high_Vector[i] = u_high_right;
            x_ref[i] = dcmYSteady;
        }
    }
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Nsteps, Nsteps) * 1.0 * 1e3;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Nsteps, Nsteps) * 1.0 * 1e-3;
    Eigen::MatrixXd eigen_qp_H = R + B_qp.transpose() * Q * B_qp;
    Eigen::MatrixXd eigen_qp_A = B_qp;
    Eigen::MatrixXd eigen_qp_g = B_qp.transpose() * Q * (A_qp * x0 - x_ref);
    Eigen::MatrixXd eigen_qp_lbA = x_low_Vector - A_qp * x0;
    Eigen::MatrixXd eigen_qp_ubA = x_high_Vector - A_qp * x0;
    Eigen::MatrixXd eigen_qp_lb = u_low_Vector;
    Eigen::MatrixXd eigen_qp_ub = u_high_Vector;

    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //       lb<=x<=ub
    qpOASES::real_t qp_H[Nsteps*Nsteps];
    qpOASES::real_t qp_A[Nsteps*Nsteps];
    qpOASES::real_t qp_g[Nsteps];
    qpOASES::real_t qp_lbA[Nsteps];
    qpOASES::real_t qp_ubA[Nsteps];
    qpOASES::real_t qp_lb[Nsteps];
    qpOASES::real_t qp_ub[Nsteps];

    copy_Eigen_to_real_t(qp_H, eigen_qp_H, eigen_qp_H.rows(), eigen_qp_H.cols());
    copy_Eigen_to_real_t(qp_A, eigen_qp_A, eigen_qp_A.rows(), eigen_qp_A.cols());
    copy_Eigen_to_real_t(qp_g, eigen_qp_g, eigen_qp_g.rows(), eigen_qp_g.cols());
    copy_Eigen_to_real_t(qp_lbA, eigen_qp_lbA, eigen_qp_lbA.rows(), eigen_qp_lbA.cols());
    copy_Eigen_to_real_t(qp_ubA, eigen_qp_ubA, eigen_qp_ubA.rows(), eigen_qp_ubA.cols());
    copy_Eigen_to_real_t(qp_lb, eigen_qp_lb, eigen_qp_lb.rows(), eigen_qp_lb.cols());
    copy_Eigen_to_real_t(qp_ub, eigen_qp_ub, eigen_qp_ub.rows(), eigen_qp_ub.cols());    

    auto qp_problem = QProblem(Nsteps, Nsteps, qpOASES::HST_UNKNOWN,
                               qpOASES::BT_TRUE);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp_problem.setOptions(options);

    int max_solver_iter = 100;

    qp_problem.init(qp_H, qp_g, qp_A, qp_lb,
                    qp_ub, qp_lbA, qp_ubA, max_solver_iter,
                    nullptr);

    std::vector<qpOASES::real_t> qp_sol(Nsteps, 0);
    qp_problem.getPrimalSolution(qp_sol.data());
    Eigen::VectorXd footplacements_Y = Eigen::VectorXd::Zero(Nsteps);
    for (int i = 0; i < Nsteps; ++i) {
        footplacements_Y[i] = qp_sol[i];
    }
    return footplacements_Y;
    
}

void FootHoldPlanner::getStanceFootSequence(int Nsteps, int currentStanceFoot){
    stanceFootSeq = Eigen::VectorXd::Zero(Nsteps);
    for(int i=0;i<Nsteps;i++){
        stanceFootSeq[i] = (currentStanceFoot+i+1) % 2;
    }
}

double FootHoldPlanner::deltaTransformation(double timeDuration){

    return exp(omega*timeDuration);
}

void copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols) {
    int count = 0;

    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count++] = std::isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}