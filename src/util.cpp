#include "util.h"
using Eigen::MatrixXd;

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

/*
MatrixXd generate_traj(std::vector<double> x0, std::vector<double> us, System S)
{
    int N = S.getNSeg() - 1;
}
 */