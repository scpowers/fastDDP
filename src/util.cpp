#include "util.h"
#include <algorithm>
#include <iterator>
using Eigen::MatrixXd;
using Eigen::VectorXd;

obstacle newOverheadHemisphere(std::string str, double loc[], double r)
{
    obstacle obs = {str, {loc[0], loc[1], 0.0}, r};
    return obs;
}

obstacle newXYCylinder(std::string str, double loc[], double r)
{
    obstacle obs = {str, {loc[0], loc[1], loc[2]}, r};
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