#include "util.h"

obstacle newOverheadHemisphere(std::string str, const double loc[], double r) {
    obstacle obs = {str, {loc[0], loc[1], 0.0}, r};
    return obs;
}