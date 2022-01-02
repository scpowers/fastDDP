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
    int k = 0;
    std::vector<double> x;
    std::vector<double> u;
    // needs access to S's variables as well (self)
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
    //std::function<auto(_some_local_struct_)> f;
    std::function<f_out(func_in in_struct)> f_func;
    //std::function<auto(_some_local_struct_)> L_func;
    //std::function<auto(_some_local_struct_)> Lf_func;
    double mu;
    std::vector<double> xd;
    std::vector<double> xmin;
    std::vector<double> xmax;
    double lambda;
    // some struct vector to hold some obstacles
    double ko;
    std::vector<double> umin;
    std::vector<double> umax;

    std::function<f_out(func_in)> getf() {return f_func;}

public:
    void setf(std::function<f_out(func_in)> func) { f_func = func; }
    f_out f(func_in in_struct) {std::function<f_out(func_in)> fcall = getf(); return fcall(in_struct);}
};


#endif //FASTDDP_SYSTEM_H
