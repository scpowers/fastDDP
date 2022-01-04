#ifndef FASTDDP_UTIL_H
#define FASTDDP_UTIL_H
#include "System.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

obstacle newOverheadHemisphere(VectorXd, double, System);

obstacle newXYCylinder(VectorXd, double);

MatrixXd generate_traj(traj_in);

double traj_cost(traj_cost_in);

#endif //FASTDDP_UTIL_H
