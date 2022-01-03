#include "System.h"

// dynamics definition
f_out System::f_func(func_in in_struct)
{
    double c = cos(in_struct.x(3));
    double s = sin(in_struct.x(3));
    double v = in_struct.x(4);
    double w = in_struct.x(5);
    MatrixXd A(in_struct.x.size(), in_struct.x.size());
    A << 1, 0, 0, -dt*s*v, dt*c, 0, 0, 0,
         0, 1, 0, dt*c*v, dt*s, 0, 0, 0,
         0, 0, 1, 0, 0, 0, dt, 0,
         0, 0, 0, 1, 0, dt, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 1;

    MatrixXd B(in_struct.x.size(), in_struct.u.size());
    B << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0,
         dt, 0, 0, 0,
         0, dt, 0, 0,
         0, 0, dt, 0,
         0, 0, 0, dt;

    VectorXd x(in_struct.x.size()); // make same size as input x
    x << in_struct.x(0) + dt*c*v,
         in_struct.x(1) + dt*s*v,
         in_struct.x(2) + dt*in_struct.x(6),
         in_struct.x(3) + dt*w,
         v + dt*in_struct.u(0),
         w + dt*in_struct.u(1),
         in_struct.x(6) + dt*in_struct.u(2),
         in_struct.x(7) + dt*in_struct.u(3);

    // initialize return struct
    f_out ret;
    ret.x = x;
    ret.A = A;
    ret.B = B;
    return ret;
}

// loss function definition
L_out System::L_func(func_in in_struct)
{
    L_out ret;
    ret.L = in_struct.u.sum();
    return ret;
}
