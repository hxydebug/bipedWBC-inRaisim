#ifndef __ADAPTIVEFOOTPLACE_H
#define __ADAPTIVEFOOTPLACE_H

class FootHoldPlanner{

public:

    FootHoldPlanner(double comHeight, double stepPeriod, double averageSpeed, double stepWidth);
    Eigen::VectorXd ComputeNextfootHold(int Nsteps,
                                        double dcmOffsetX,
                                        double dcmOffsetY,
                                        int currentStancefootID,
                                        double currentStancefootPositionX,
                                        double currentStancefootPositionY,
                                        double phase);
    double deltaTransformation(double timeDuration);
    void getStanceFootSequence(int Nsteps, int currentStanceFoot);
    double dcmXSteady;
    double dcmYSteady;
    Eigen::VectorXd stanceFootSeq;
    Eigen::MatrixXd A_qp;
    Eigen::MatrixXd B_qp;
    double omega; // frequency for biped walking

private:
    double g; // 9.8
    double h; // com height
    double stepDuration; // time length of each step
    double leftoverTime; // the time duration before this step finishes
    double averageSpeedX; // average longitudinal speed
    double stepLengthSteady;
    double stepWidthSteady;
    
};