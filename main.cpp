/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include "System.h"

using Eigen::MatrixXd;

int main()
{
    // checking attributes of S
    System S;
    // adding a simple lambda func for f attribute (dynamics)
    std::function<float(float)> f = [] (float x) {return 2.5*x;};
    S.setf(f);
    float y = S.f(3);
    std::cout << "3 * 2.5 = " << y << std::endl;

    return 0;
}
