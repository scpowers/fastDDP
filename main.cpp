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
    S.setNSeg(4);

    // testing dynamics
    func_in func_in_test;
    VectorXd x(3);
    x << 1.5, 3.5, 4.5;
    std::cout << x << std:: endl;
    func_in_test.x = x;
    std::cout << func_in_test.x.size() << std::endl;
    f_out y = S.f(func_in_test);
    std::cout << y.x(0) << std::endl;

    // testing loss
    L_out z = S.L(func_in_test);
    std::cout << z.L << std::endl;

    // testing overhead hemisphere obstacles
    double loc1[2] = {3.5, 2.5};
    obstacle o1 = newOverheadHemisphere("OverheadHemisphere", loc1, 1.0);
    S.addObs(o1);
    std::cout << S.getObsAt(0).type << std::endl;
    std::cout << S.getObsAt(0).r << std::endl;

    // testing overhead hemisphere obstacles
    double loc2[3] = {3.5, 2.5, 3.5};
    obstacle o2 = newXYCylinder("XYCylinder", loc2, 1.5);
    S.addObs(o2);
    std::cout << S.getObsAt(1).type << std::endl;
    std::cout << S.getObsAt(1).r << std::endl;

    // testing trajectories
    MatrixXd us(2, S.getNSeg() - 1);
    traj_in input = {x, us, S};
    MatrixXd traj = generate_traj(input);
    std::cout << traj << std::endl;

    // testing trajectory cost
    traj_cost_in cost_in = {traj, us, S};
    double J = traj_cost(cost_in);
    std::cout << J << std::endl;

    return 0;
}
