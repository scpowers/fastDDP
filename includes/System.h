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
    std::vector<float> x;
    MatrixXd A;
    MatrixXd B;
};

// define input struct for dynamics (f) and loss (L)

class System {
private:
    float Ceiling_H;
    float tf;
    int N_seg;
    float dt;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd Qf;
    //std::function<auto(_some_local_struct_)> f;
    std::function<f_out(float x)> f_func;
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

    std::function<f_out(float)> getf() {return f_func;}

public:
    void setf(std::function<f_out(float)> func) { f_func = func; }
    f_out f(float x) {std::function<f_out(float)> fcall = getf(); return fcall(x);}
};


#endif //FASTDDP_SYSTEM_H
