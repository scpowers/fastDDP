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
    double L;
    MatrixXd Lx;
    MatrixXd Lxx;
    MatrixXd Lu;
    MatrixXd Luu;
};

class System {
private:
    double Ceiling_H;
    double tf;
    int N_seg;
    double dt;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd Qf;
    f_out f_func(func_in in_struct);
    L_out L_func(func_in in_struct);

    double mu;
    std::vector<double> xd;
    std::vector<double> xmin;
    std::vector<double> xmax;
    double lambda;
    // some struct vector to hold some obstacles
    double ko;
    std::vector<double> umin;
    std::vector<double> umax;

public:
    f_out f(func_in in_struct) {return f_func(in_struct);}
    L_out L(func_in in_struct) {return L_func(in_struct);}
    void setTf(double new_tf) {tf = new_tf;}
    void setN(int new_N) {N_seg = new_N; if (tf!=0) dt=tf/N_seg;}
};


#endif //FASTDDP_SYSTEM_H
