/*
 * Main file to test out the DDP engine
 */

#include <iostream>
#include <Eigen/Dense>
#include <sciplot/sciplot.hpp>
#include "includes/System.h"
#include "includes/util.h"
#include "DDP_Engine.h"

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
    VectorXd xmin {{-10, -10, 1, -pi, -1, -pi, -1, -pi}};
    S.setXmin(xmin);
    VectorXd xmax {{10, 10, S.getCH(), pi, 1, pi, 1, pi}};
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
    VectorXd loc1(2);
    loc1 << -2, -3;
    double r1 = 1.0;
    obstacle o1 = newXYCylinder(loc1, r1);
    S.addObs(o1);

    // define initial control sequence
    MatrixXd us(umax.size(), S.getNSeg() - 1);
    us.setZero();

    // trajectory and cost associated with initial control sequence
    traj_in input = {x0, us, S};
    MatrixXd xs = generate_traj(input);

    traj_cost_in cost_in = {xs, us, S};
    double J = traj_cost(cost_in);
    cout << "initial trajectory cost: " << J << endl;

    DDP_Engine ddp;
    int numRuns = 50;
    VectorXd JVec(numRuns);
    // plot x and y over the trajectory
    plt::Plot plot;
    VectorXd time_vec(S.getNSeg());
    time_vec.setLinSpaced(S.getNSeg(), 0, S.getTf());
    for (int i = 0; i < numRuns; i++)
    {
        traj_in ddp_in = {x0, us, S};
        ddp_out ddpOut = ddp.run(ddp_in);
        //cout << "\n" << ddpOut.V << endl;
        //cout << "\n" << ddpOut.Vn << endl;
        //cout << "\n" << ddpOut.a << endl;
        //cout << "\n" << ddpOut.dV << endl;
        //cout << "\n" << ddpOut.dus << endl;

        // update controls
        cout << "\n*******************************" << endl;
        us = us + ddpOut.dus;
        //cout << "us: " << us << "\n" << endl;
        // update trajectory
        traj_in newTraj = {x0, us, S};
        xs = generate_traj(newTraj);

        // compute and store cost for this run
        traj_cost_in newTrajCostIn = {xs, us, S};
        JVec(i) = traj_cost(newTrajCostIn);

        // update plot
        plot.drawCurve(xs.row(0), xs.row(1)).lineColor("blue");
    }

    cout << "optimal cost:" << JVec(numRuns-1) << endl;

    // draw optimal trajectory
    plot.drawCurve(xs.row(0), xs.row(1)).lineColor("green");
    plot.xlabel("x");
    plot.ylabel("y");
    plot.xrange(-6, 4);
    plot.yrange(-6, 1);
    plot.fontName("Palatino");
    plot.legend().hide();
    plot.show();

    // plot z over time
    plt::Plot plot2;
    plot2.drawCurve(time_vec, xs.row(2));
    plot2.xlabel("time");
    plot2.ylabel("z");
    plot2.xrange(0.0, S.getTf());
    plot2.fontName("Palatino");
    plot2.legend().hide();
    plot2.show();

    // plot controls over time
    plt::Plot plot3;
    plot3.drawCurve(time_vec, us.row(0)).label("ua");
    plot3.drawCurve(time_vec, us.row(1)).label("ud");
    plot3.drawCurve(time_vec, us.row(2)).label("uz");
    plot3.drawCurve(time_vec, us.row(3)).label("uphi");
    plot3.xlabel("time");
    plot3.ylabel("u");
    plot3.xrange(0.0, S.getTf());
    plot3.fontName("Palatino");
    plot3.show();


    return 0;
}
