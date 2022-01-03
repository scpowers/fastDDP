#include "util.h"
#include <iterator>
using Eigen::MatrixXd;
using Eigen::VectorXd;

obstacle newOverheadHemisphere(std::string str, double loc[], double r, System S)
{
    obstacle obs = {str, {loc[0], loc[1], S.getCH()}, r};
    return obs;
}

obstacle newXYCylinder(std::string str, double loc[], double r)
{
    obstacle obs = {str, {loc[0], loc[1], 0.0}, r};
    return obs;
}

MatrixXd generate_traj(traj_in in_struct)
{
    MatrixXd xs(in_struct.x0.size(), in_struct.S.getNSeg()); // initialize xs
    xs.col(0) = in_struct.x0; // know x0 already, fill in rest of time steps

    int N = in_struct.S.getNSeg() - 1;
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

    for (int k = 0; k < NSeg; k++)
    {
        func_in in;
        if (k < NSeg-1) // when a control signal exists
            in = {k, in_struct.xs.col(k), in_struct.us.col(k)};
        else // after the last control signal
        {
            VectorXd u(in_struct.xs.col(k).size());
            u.setZero();
            in = {k, in_struct.xs.col(k), u};
        }
        L_out out = in_struct.S.L(in);
        J = J + out.L; // update running cost
    }
    return J;
}