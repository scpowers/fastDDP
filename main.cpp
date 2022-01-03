/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include "includes/System.h"
#include "includes/util.h"

using Eigen::MatrixXd;

const double pi = 3.14159265358979323846;

int main()
{
    // checking attributes of S
    System S;
    S.setTf(40);
    S.setNSeg(64);

    // define an x0
    VectorXd x0 {{-5, -5, 2.8, 0, 0, 0, 0, pi}};

    // add an XY cylinder
    double loc1[3] = {-2, -3};
    obstacle o1 = newXYCylinder("XYCylinder", loc1, 1);
    S.addObs(o1);

    // testing trajectories
    MatrixXd us(2, S.getNSeg() - 1);
    us.setOnes();
    traj_in input = {x0, us, S};
    MatrixXd traj = generate_traj(input);

    // testing trajectory cost
    traj_cost_in cost_in = {traj, us, S};
    double J = traj_cost(cost_in);
    std::cout << "sample trajectory cost: " << J << std::endl;

    return 0;
}
