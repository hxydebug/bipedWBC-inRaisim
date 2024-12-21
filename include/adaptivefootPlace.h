#ifndef __ADAPTIVEFOOTPLACE_H
#define __ADAPTIVEFOOTPLACE_H

class FootHoldPlanner{

public:

    FootHoldPlanner(double comHeight, double stepDuration, double stepWidth);
    std::vector<double> ComputeNextfootHold(
        int Nsteps,
        Eigen::VectorXd b0,
        int currentStancefootID,
        Eigen::VectorXd currentStancefootPosition,
        double phase);

private:
    float g; // 9.8
    float h; // com height
    float omega; // frequency for biped walking
    float stepDuration; // time length of each step
    float leftoverTime; // the time duration before this step finishes
    float averageSpeed; // average longitudinal speed
    
};