#include "System.h"

// dynamics definition
f_out System::f_func(func_in in_struct)
{
    double tmp = in_struct.x(0);

    // initialize return struct
    f_out ret;
    VectorXd x(in_struct.x.size()); // make same size as input x
    x(0) = tmp+dt;
    ret.x = x;
    return ret;
}

// loss function definition
L_out System::L_func(func_in in_struct)
{
    L_out ret;
    ret.L = 25 + dt;
    return ret;
}
