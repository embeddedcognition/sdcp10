/*
############################################################
## AUTHOR: James Beasley                                  ##
## DATE: August 13, 2017                                  ##
## UDACITY SDC: Project 10 (Model Predictive Controllers) ##
############################################################
*/

#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
    public:
        MPC();
        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
