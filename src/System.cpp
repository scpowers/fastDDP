#include "System.h"
#include <cmath>
#include <iostream>

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
    VectorXd xfError(in_struct.x.size());
    double L;
    VectorXd Lx(in_struct.x.size());
    MatrixXd Lxx(in_struct.x.size(), in_struct.x.size());
    VectorXd Lu(in_struct.u.size());
    MatrixXd Luu(in_struct.u.size(), in_struct.u.size());
    if (in_struct.k == NSeg) // add terminal cost if at end
    {
        xfError = in_struct.x - xd;
        L = 0.5*xfError.transpose()*Qf*xfError;
        Lx = Qf*xfError;
        Lxx = Qf;
        Lu.setZero();
        Luu.setZero();
    }
    else // accumulate running cost if en route
    {
        L = in_struct.x.transpose()*Q*in_struct.x;
        L = L + in_struct.u.transpose()*R*in_struct.u;
        L = L * 0.5 * dt;
        Lx = dt*Q*in_struct.x;
        Lxx = dt*Q;
        Lu = dt*R*in_struct.u;
        Luu = dt*R;
    }

    // now add quadratic penalty term for obstacle collisions
    if (!obs.empty()) // if you have obstacles, that is
    {
        for (auto o : obs) // loop over obstacles
        {
            if (o.type.compare("XYCylinder") == 0)
            {
                // just interested in xy geometric difference, no z for now
                VectorXd g = in_struct.x.head(2) - o.loc.head(2);
                double c = o.r - g.norm();
                if (c >= 0) // meaning a collision has occurred
                {
                    L = L + 0.5*ko*pow(c,2);
                    VectorXd v = g/g.norm();
                    Lx.head(2) = Lx.head(2) - ko*c*v;
                    Lxx.block(0,0,2,2) = Lxx.block(0,0,2,2) + ko*v*v.transpose();
                }
            }
        }
    }

    // now penalize state constraint infractions
    for (int i = 0; i < xmin.size(); i++)
    {
        if (in_struct.x(i) < xmin(i))
            L = L + lambda*(xmin(i) - in_struct.x(i));
        else if (in_struct.x(i) > xmax(i))
            L = L + lambda*(in_struct.x(i) - xmax(i));
        else {}
    }

    // initialize and populate returned struct
    L_out ret;
    ret.L = L;
    ret.Lx = Lx;
    ret.Lxx = Lxx;
    ret.Lu = Lu;
    ret.Luu = Luu;
    return ret;
}
