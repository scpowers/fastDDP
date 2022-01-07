#include "util.h"
#include <iterator>
#include <string>
using Eigen::MatrixXd;
using Eigen::VectorXd;

obstacle newOverheadHemisphere(VectorXd loc, double r, System S)
{
    std::string str = "OverheadHemisphere";
    VectorXd newVec(3);
    newVec << loc(0), loc(1), S.getCH();
    obstacle obs = {str, newVec, r};
    return obs;
}

obstacle newXYCylinder(VectorXd loc, double r)
{
    std::string str = "XYCylinder";
    VectorXd newVec(3);
    newVec << loc(0), loc(1), 0.0;
    obstacle obs = {str, newVec, r};
    return obs;
}

MatrixXd generate_traj(traj_in in_struct)
{
    MatrixXd xs(in_struct.x0.size(), in_struct.S.getNSeg()+1); // initialize xs
    xs.col(0) = in_struct.x0; // know x0 already, fill in rest of time steps

    int N = in_struct.S.getNSeg();
    for (int i = 0; i < N; i++)
    {
        func_in in = {0, xs.col(i), in_struct.us.col(i)};
        f_out out = in_struct.S.f(in);
        xs.col(i+1) = out.x;
    }

    return xs;
}

double traj_cost(traj_cost_in in_struct)
{
    double J = 0; // initialize cost
    int NSeg = in_struct.S.getNSeg();

    for (int k = 0; k < NSeg+1; k++)
    {
        func_in in;
        if (k < NSeg) // when a control signal exists
            in = {k, in_struct.xs.col(k), in_struct.us.col(k)};
        else // after the last control signal
        {
            VectorXd u(in_struct.us.col(0).size());
            u.setZero();
            in = {k, in_struct.xs.col(k), u};
        }
        L_out out = in_struct.S.L(in);
        J = J + out.L; // update running cost
    }
    return J;
}