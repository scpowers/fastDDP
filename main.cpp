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
    S.setTf(10.0);
    S.setN(4);

    func_in func_in_test;
    func_in_test.x.push_back(3.5);
    f_out y = S.f(func_in_test);
    std::cout << y.x.at(0) << std::endl;

    L_out z = S.L(func_in_test);
    std::cout << z.L << std::endl;

    double loc1[2] = {3.5, 2.5};
    obstacle o1 = newOverheadHemisphere("OverheadHemisphere", loc1, 1.0);
    S.addObs(o1);
    std::cout << S.getObsAt(0).type << std::endl;

    return 0;
}
