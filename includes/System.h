#ifndef FASTDDP_SYSTEM_H
#define FASTDDP_SYSTEM_H
#include <Eigen/Dense>
#include <string>
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
    VectorXd loc;
    double r;
};

class System {
protected:
    double Ceiling_H = 3.5; // ceiling height
    double tf = 40; // time horizon
    int NSeg = 5; // number of time segments
    double dt = tf/NSeg; // time step
    MatrixXd Q; // running cost matrix on states
    MatrixXd R; // running cost matrix on controls
    MatrixXd Qf; // terminal cost matrix on states
    f_out f_func(func_in in_struct); // dynamics
    L_out L_func(func_in in_struct); // loss

    double mu = 0;
    VectorXd xd; // desired states at end
    VectorXd xmin; // minimum values for states
    VectorXd xmax; // maximum values for states
    double lambda = 1e4;
    std::vector<obstacle> obs; // vector of obstacles
    double ko = 1e5;
    VectorXd umin; // minimum values for controls
    VectorXd umax; // maximum values for controls

public:
    // publicly call the dynamics
    f_out f(func_in in_struct) {return f_func(in_struct);}
    // publicly call the loss
    L_out L(func_in in_struct) {return L_func(in_struct);}

    // setters
    void setTf(double new_tf) {tf = new_tf; dt=tf/NSeg;}
    void setNSeg(int new_N) {NSeg = new_N; dt=tf/NSeg;}
    void addObs(obstacle new_obs) {obs.push_back(new_obs);}
    void setXd(VectorXd newXd) {xd = newXd;}
    void setXmin(VectorXd newXmin) {xmin = newXmin;}
    void setXmax(VectorXd newXmax) {xmax = newXmax;}
    void setUmin(VectorXd newUmin) {umin = newUmin;}
    void setUmax(VectorXd newUmax) {umax = newUmax;}
    void setQ(MatrixXd newQ) {Q = newQ;}
    void setQf(MatrixXd newQf) {Qf = newQf;}
    void setR(MatrixXd newR) {R = newR;}


    // getters
    obstacle getObsAt(int i) {return obs.at(i);}
    int getNSeg() {return NSeg;}
    double getTf() {return tf;}
    double getCH() {return Ceiling_H;}
    MatrixXd getQ() {return Q;}
    MatrixXd getQf() {return Qf;}
    MatrixXd getR() {return R;}
};

// define input struct for generate_traj
struct traj_in {
    VectorXd x0;
    MatrixXd us;
    System S;
};

// define input struct for traj_cost
struct traj_cost_in {
    MatrixXd xs;
    MatrixXd us;
    System S;
};
#endif //FASTDDP_SYSTEM_H
