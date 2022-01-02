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
    std::function<int(int x)> f;
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

public:
    void setf(std::function<int(int)> func) { f = func; }
    std::function<int(int)> getf() {return f;}
};


#endif //FASTDDP_SYSTEM_H
