/*
 * This is a C++ implementation of a MATLAB DDP engine written
 * by Dr. Marin Kobilarov at Johns Hopkins University. I took
 * his course on applied optimal control and wanted to rewrite
 * this in C++ to better my understanding of both DDP and C++.
 * All credit goes to Dr. Kobilarov. His contact info is:
 * marin(at)jhu.edu.
 *
 * Some of his documentation comments are transcribed below
 * for clarity:
 *
 * Summary: This computes second-order numerical optimal control.
 * The code computes the optimal control adjustment for a given
 * dynamical system.
 *
 * In this implementation, second-order terms in the dynamics are
 * ignored, which corresponds to the linear-quadratic-subproblem
 * (LQS) approach.
 */

#ifndef FASTDDP_DDP_ENGINE_H
#define FASTDDP_DDP_ENGINE_H
#include "util.h"
#include "System.h"

// define struct to return from DDP
struct ddp_out {
    MatrixXd dus; //mxN matrix containing computed optimal change in control
    double V; // current value function
    double Vn; // new value function
    VectorXd dV; // predicted change in value function
    double a; // step size along control search direction
};

class DDP_Engine {
protected:


public:
    ddp_out run(traj_in); // main function called on DDP_engine object
    // add backward pass func called inside run
    // add forward pass func called inside run
};


#endif //FASTDDP_DDP_ENGINE_H
