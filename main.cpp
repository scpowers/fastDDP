/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include <sciplot/sciplot.hpp>
#include "includes/System.h"
#include "includes/util.h"

using std::cout;
using std::endl;
namespace plt = sciplot;

const double pi = 3.14159265358979323846;

int main()
{
    // setting time horizon and discretization
    System S;
    S.setTf(40);
    S.setNSeg(64);

    // define state and control vectors
    VectorXd x0 {{-5, -5, 2.8, 0, 0, 0, 0, pi}};
    cout << "x0:\n" << x0.transpose() << endl;
    VectorXd xd {{4, 0, 3, 0, 0, 0, 0, pi/2}};
    S.setXd(xd);
    VectorXd xmin {{-10, -10, 1, -2*pi, -5, -2*pi, -1, -2*pi}};
    S.setXmin(xmin);
    VectorXd xmax {{10, 10, S.getCH(), 2*pi, 5, 2*pi, 1, 2*pi}};
    S.setXmax(xmax);
    VectorXd umin {{-0.1, -0.1, -0.5, -0.1}};
    S.setUmin(umin);
    VectorXd umax {{0.1, 0.1, 0.5, 0.1}};
    S.setUmax(umax);

    // define cost function matrices
    VectorXd Q_diag(x0.size());
    Q_diag << 0, 0, 0, 0, 0, 0, 0, 0;
    MatrixXd Q = Q_diag.asDiagonal();
    S.setQ(Q);

    VectorXd Qf_diag(x0.size());
    Qf_diag << 100, 100, 50, 1, 1, 1, 1, 10;
    MatrixXd Qf = Qf_diag.asDiagonal();
    S.setQf(Qf);

    VectorXd R_diag(umax.size());
    R_diag << 1, 1, 1, 1;
    MatrixXd R = 0.5*R_diag.asDiagonal();
    S.setR(R);

    // add an XY cylinder
    double loc1[3] = {-2, -3};
    double r1 = 1.0;
    obstacle o1 = newXYCylinder("XYCylinder", loc1, r1);
    S.addObs(o1);

    // define initial control sequence
    MatrixXd us(umax.size(), S.getNSeg() - 1);
    us.setZero();

    // trajectory and cost associated with initial control sequence
    traj_in input = {x0, us, S};
    MatrixXd xs = generate_traj(input);

    traj_cost_in cost_in = {xs, us, S};
    double J = traj_cost(cost_in);
    cout << "sample trajectory cost: " << J << endl;

    plt::Plot plot;


    return 0;
}
