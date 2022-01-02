//
// Created by Spencer Powers on 12/27/21.
//

#ifndef FASTDDP_SYSTEM_H
#define FASTDDP_SYSTEM_H
#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;

// define output struct for dynamics (f)
struct f_out {
    std::vector<double> x;
    MatrixXd A;
    MatrixXd B;
};

// define input struct for dynamics (f) and loss (L)
struct func_in {
    int k = 0; // initialize to 0 but will be set in ddp engine call
    std::vector<double> x;
    std::vector<double> u;
    // needs access to S's variables as well (self)
};

// define output struct for loss (L)
struct L_out {
    double L; // cost value
    MatrixXd Lx; // grad_x L
    MatrixXd Lxx; // grad^2_x L
    MatrixXd Lu; // grad_u L
    MatrixXd Luu; // grad^2_u L
};

class System {
private:
    double Ceiling_H; // ceiling height
    double tf; // time horizon
    int N_seg; // number of time segments
    double dt; // time step
    MatrixXd Q; // running cost matrix on states
    MatrixXd R; // running cost matrix on controls
    MatrixXd Qf; // terminal cost matrix on states (or state errors)
    f_out f_func(func_in in_struct); // dynamics
    L_out L_func(func_in in_struct); // loss

    double mu;
    std::vector<double> xd; // desired states at end
    std::vector<double> xmin; // minimum values for states
    std::vector<double> xmax; // maximum values for states
    double lambda;
    // some struct vector to hold some obstacles
    double ko;
    std::vector<double> umin; // minimum values for controls
    std::vector<double> umax; // maximum values for controls

public:
    // publicly call the dynamics
    f_out f(func_in in_struct) {return f_func(in_struct);}
    // publicly call the loss
    L_out L(func_in in_struct) {return L_func(in_struct);}
    // setters
    void setTf(double new_tf) {tf = new_tf;}
    void setN(int new_N) {N_seg = new_N; if (tf!=0) dt=tf/N_seg;}
};


#endif //FASTDDP_SYSTEM_H
