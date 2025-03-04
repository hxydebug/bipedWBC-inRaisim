#ifndef __ADAPTIVEFOOTPLACE_H
#define __ADAPTIVEFOOTPLACE_H
#include "qpOASES.hpp"
#include "qpOASES/Types.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "Eigen/SparseCore"
#include <math.h>

class FootHoldPlanner{

public:

    FootHoldPlanner(double comHeight, double stepPeriod, double averageSpeed, double stepWidth);
    Eigen::VectorXd ComputeNextfootHold(int Nsteps,
                                        double dcmOffsetX,
                                        double dcmOffsetY,
                                        int currentStancefootID,
                                        double currentStancefootPositionX,
                                        double currentStancefootPositionY,
                                        double phase,
                                        double comHeight,
                                        double stepPeriod,
                                        double averageSpeed);
    double deltaTransformation(double timeDuration);
    void getStanceFootSequence(int Nsteps, int currentStanceFoot);
    Eigen::VectorXd optimalLongitudinalFootPlacement(int Nsteps, double dcmOffsetX);
    Eigen::VectorXd optimalLateralFootPlacement(int Nsteps, double dcmOffsetY);
    double dcmXSteady;
    double dcmYSteady;
    double current_dcmoffset_X;
    double current_dcmoffset_Y;
    Eigen::VectorXd stanceFootSeq;
    Eigen::MatrixXd A_qp;
    Eigen::MatrixXd B_qp;
    Eigen::MatrixXd M;
    Eigen::MatrixXd MM;
    Eigen::MatrixXd NN;
    double cbf_gamma;
    double omega; // frequency for biped walking
    double leftoverTime; // the time duration before this step finishes
    Eigen::VectorXd footplacements_Xs;
    Eigen::VectorXd footplacements_Ys;
    Eigen::VectorXd dcmXs;
    Eigen::VectorXd dcmYs;
    int currentStancefoot_ID;
    double currentStancefootPosition_X;
    double currentStancefootPosition_Y;
    double averageSpeedX; // average longitudinal speed
    double stepDuration; // time length of each step

private:
    double g; // 9.8
    double h; // com height
    double stepLengthSteady;
    double stepWidthSteady;
    
};

#endif