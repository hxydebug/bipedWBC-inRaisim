#include "adaptivefootPlace.h"
#include "qpOASES.hpp"
#include "qpOASES/Types.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "Eigen/SparseCore"

using qpOASES::QProblem;
void WBC_priority::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols);


FootHoldPlanner::FootHoldPlanner(double comHeight, double stepDuration, double stepWidth)
{


}

Eigen::VectorXd ComputeNextfootHold(int Nsteps,
                                        Eigen::VectorXd b0,
                                        int currentStancefootID,
                                        Eigen::VectorXd currentStancefootPosition,
                                        double phase)
{
    
}

void WBC_priority::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols) {
    int count = 0;

    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count++] = std::isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}