#include "System.h"

// dynamics definition
f_out System::f_func(func_in in_struct) {
    double tmp = in_struct.x.at(0);
    f_out ret;
    ret.x.push_back(tmp+dt);
    return ret;
}

// loss function definition
L_out System::L_func(func_in in_struct) {

    L_out ret;
    ret.L = 25 + dt;
    return ret;
}
