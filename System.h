//
// Created by Spencer Powers on 12/27/21.
//

#ifndef FASTDDP_SYSTEM_H
#define FASTDDP_SYSTEM_H
#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;

class System {
private:
    float Ceiling_H;
    float tf;
    float N_seg;
    float dt;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd Qf;
    //std::function<auto(_some_local_struct_)> f;
    std::function<float(float x)> f_func;
    //std::function<auto(_some_local_struct_)> L;
    //std::function<auto(_some_local_struct_)> Lf;
    float mu;
    std::vector<float> xd;
    std::vector<float> xmin;
    std::vector<float> xmax;
    float lambda;
    // some struct vector to hold some obstacles
    float ko;
    std::vector<float> umin;
    std::vector<float> umax;

    std::function<float(float)> getf() {return f_func;}

public:
    void setf(std::function<float(float)> func) { f_func = func; }
    float f(float x) {std::function<float(float)> fcall = getf(); return fcall(x);}
};


#endif //FASTDDP_SYSTEM_H
