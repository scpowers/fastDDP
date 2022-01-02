/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include "includes/System.h"
#include "includes/util.h"

using Eigen::MatrixXd;

int main()
{
    // checking attributes of S
    System S;
    // adding a simple lambda func for f attribute (dynamics)
    S.setf(f);
    f_out y = S.f(4);
    std::cout << y.x.at(0) << std::endl;

    return 0;
}
