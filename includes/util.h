#ifndef FASTDDP_UTIL_H
#define FASTDDP_UTIL_H
#include "System.h"

using Eigen::MatrixXd;

obstacle newOverheadHemisphere(std::string, double[], double);

obstacle newXYCylinder(std::string, double[], double);

MatrixXd generate_traj(std::vector<double>(), std::vector<double>(), System);

#endif //FASTDDP_UTIL_H
