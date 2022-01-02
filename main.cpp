/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include "System.h"

using Eigen::MatrixXd;

int main()
{
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    System S;

    std::function<int(int)> f = [&] (int x) {return 2*x;};
    S.setf(f);
    std::function<int(int)> fcheck = S.getf();
    int y = fcheck(3);

    return 0;
}
