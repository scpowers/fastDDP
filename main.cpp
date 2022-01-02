/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include "System.h"

using Eigen::MatrixXd;

int main()
{
    // checking Eigen imported correctly
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    // checking attributes of S
    System S;
    // adding a simple lambda func for f attribute (dynamics)
    std::function<int(int)> f = [&] (int x) {return 2*x;};
    S.setf(f);
    std::function<int(int)> fcheck = S.getf();
    int y = fcheck(3);
    std::cout << y << std::endl;

    return 0;
}
