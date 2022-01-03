#include "System.h"

// dynamics definition
f_out System::f_func(func_in in_struct)
{
    // initialize return struct
    f_out ret;
    VectorXd x(in_struct.x.size()); // make same size as input x
    x = in_struct.x;
    x.head(in_struct.u.size()) = x.head(in_struct.u.size()) +
            in_struct.u;
    ret.x = x;
    return ret;
}

// loss function definition
L_out System::L_func(func_in in_struct)
{
    L_out ret;
    ret.L = in_struct.u.sum();
    return ret;
}
