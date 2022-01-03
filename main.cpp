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

    // define an x0
    VectorXd x0(3);
    x0 << 1.5, 3.5, 4.5;
    std::cout << "x0:\n" << x0 << std:: endl;

    // add an XY cylinder
    double loc1[3] = {3.5, 2.5, 3.5};
    obstacle o1 = newXYCylinder("XYCylinder", loc1, 1);
    S.addObs(o1);

    // testing trajectories
    MatrixXd us(2, S.getNSeg() - 1);
    us.setOnes();
    traj_in input = {x0, us, S};
    MatrixXd traj = generate_traj(input);
    std::cout << "sample trajectory:\n" << traj << std::endl;

    // testing trajectory cost
    traj_cost_in cost_in = {traj, us, S};
    double J = traj_cost(cost_in);
    std::cout << "sample trajectory cost: " << J << std::endl;

    return 0;
}
