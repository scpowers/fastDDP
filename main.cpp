/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include "System.h"
#include "util.h"

using Eigen::MatrixXd;

int main()
{
    // checking attributes of S
    System S;
    // adding a simple lambda func for f attribute (dynamics)
    S.setf(f);
    float y = S.f(4);
    std::cout << "4 * 4.5 = " << y << std::endl;

    return 0;
}
