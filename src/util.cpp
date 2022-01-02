#include "util.h"

obstacle newOverheadHemisphere(std::string str, double loc[], double r) {
    obstacle obs = {str, {loc[0], loc[1], 0.0}, r};
    return obs;
}

obstacle newXYCylinder(std::string str, double loc[], double r) {
    obstacle obs = {str, {loc[0], loc[1], loc[2]}, r};
    return obs;
}