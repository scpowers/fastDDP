#ifndef FASTDDP_SYSTEM_H
#define FASTDDP_SYSTEM_H
#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// define output struct for dynamics (f)
struct f_out {
    VectorXd x;
    MatrixXd A;
    MatrixXd B;
};

// define input struct for dynamics (f) and loss (L)
struct func_in {
    int k = 0; // initialize to 0 but will be set in ddp engine call
    VectorXd x;
    VectorXd u;
};

// define output struct for loss (L)
struct L_out {
    double L; // cost value
    MatrixXd Lx; // grad_x L
    MatrixXd Lxx; // grad^2_x L
    MatrixXd Lu; // grad_u L
    MatrixXd Luu; // grad^2_u L
};

// define struct to hold obstacles
struct obstacle {
    std::string type;
    double loc[3];
    double r;
};

class System {
private:
    double Ceiling_H; // ceiling height
    double tf; // time horizon
    int NSeg; // number of time segments
    double dt; // time step
    MatrixXd Q; // running cost matrix on states
    MatrixXd R; // running cost matrix on controls
    MatrixXd Qf; // terminal cost matrix on states (or state errors)
    f_out f_func(func_in in_struct); // dynamics
    L_out L_func(func_in in_struct); // loss

    double mu;
    VectorXd xd; // desired states at end
    VectorXd xmin; // minimum values for states
    VectorXd xmax; // maximum values for states
    double lambda;
    std::vector<obstacle> obs; // vector of obstacles
    double ko;
    VectorXd umin; // minimum values for controls
    VectorXd umax; // maximum values for controls

public:
    // publicly call the dynamics
    f_out f(func_in in_struct) {return f_func(in_struct);}
    // publicly call the loss
    L_out L(func_in in_struct) {return L_func(in_struct);}
    // setters
    void setTf(double new_tf) {tf = new_tf;}
    void setN(int new_N) {NSeg = new_N; if (tf!=0) dt=tf/NSeg;}
    void addObs(obstacle new_obs) {obs.push_back(new_obs);}

    // getters
    obstacle getObsAt(int i) {return obs.at(i);}
    int getNSeg() {return NSeg;}
};


#endif //FASTDDP_SYSTEM_H
