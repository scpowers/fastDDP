#include "DDP_Engine.h"
#include <vector>
#include <algorithm>
#include <Eigen/Cholesky>

using std::min;
using std::max;

/*
 * Returns optimal control deviations du and updated optimal cost
 * after one iteration of the DDP algorithm (not returning converged values)
 */
ddp_out DDP_Engine::run(traj_in in_struct)
{
    int n = in_struct.x0.size();
    int m = in_struct.us.rows();
    int N = in_struct.us.cols();
    if (a == -1) // if not yet used
        a = 1;

    std::vector<MatrixXd> Ps; // quantity N+1 of nxn matrices
    for (int i = 0; i < N+1; i++)
    {
        MatrixXd tmp(n, n);
        tmp.setZero();
        Ps.push_back(tmp);
    }
    MatrixXd vs(n, N+1);
    vs.setZero();
    MatrixXd cs(m, N);
    cs.setZero();
    std::vector<MatrixXd> Ds; // quantity N of mxn matrices
    for (int i = 0; i < N; i++)
    {
        MatrixXd tmp(m,n);
        tmp.setZero();
        Ds.push_back(tmp);
    }
    MatrixXd dus(in_struct.us.rows(), in_struct.us.cols());
    dus.setZero();

    // integrate trajectory and get terminal cost
    MatrixXd xs = generate_traj(in_struct);
    VectorXd tmpU(m);
    tmpU.setZero();
    func_in L_in = {N, xs.col(xs.cols()-1), tmpU };
    L_out LOut = in_struct.S.L(L_in);

    // initialize
    double V = LOut.L;
    VectorXd v = LOut.Lx;
    MatrixXd P = LOut.Lxx;

    VectorXd dV(2);
    dV.setZero();

    Ps.at(N) = P; // really slot N+1
    vs.col(N) = v; // really slow N+1

    // begin backward pass
    for (int k = N-1; k >= 0; k--)
    {
        VectorXd x = xs.col(k);
        VectorXd u = in_struct.us.col(k);

        func_in fIn = {k, x, u};
        f_out fOut = in_struct.S.f(fIn);
        L_out newLOut = in_struct.S.L(fIn);

        V = V + newLOut.L;

        MatrixXd Qx = newLOut.Lx + fOut.A.transpose()*v;
        MatrixXd Qu = newLOut.Lu + fOut.B.transpose()*v;
        MatrixXd Qxx = newLOut.Lxx + fOut.A.transpose()*P*fOut.A;
        MatrixXd Quu = newLOut.Luu + fOut.B.transpose()*P*fOut.B;
        MatrixXd Qux = fOut.B.transpose()*P*fOut.A;

        double mu = in_struct.S.getMu();
        double dmu = 1;

        // regularization while loop to ensure Quu > 0
        bool done = true;
        MatrixXd F;
        while (done == true)
        {
            MatrixXd id(m, m);
            id.setIdentity();
            MatrixXd Quum = Quu + mu*id;

            Eigen::LLT<MatrixXd> lltOfQuum(Quum);
            F = lltOfQuum.matrixL();
            F.transposeInPlace();

            if (lltOfQuum.info() == Eigen::Success)
            {
                // standard quadratic rule specified by Tassa/Todorov
                dmu = min(1/dmu0, dmu/dmu0);
                if (mu*dmu > mu0)
                    mu = mu*dmu;
                else
                    mu = mu0;

                break;
            }

            dmu = max(dmu0, dmu*dmu0);
            mu = max(mu0, mu*dmu);

            if (mu > mumax)
                break;

        }
        if (mu > mumax)
            break;

        // control law is du = c + D*dx
        MatrixXd tmp1(Qu.size(), Qux.cols()+1);
        tmp1 << Qu, Qux;
        MatrixXd tmp2 = F.transpose().colPivHouseholderQr().solve(tmp1);
        MatrixXd cD = -1.0*F.colPivHouseholderQr().solve(tmp2);
        VectorXd c = cD.col(0);
        MatrixXd D = cD.bottomRightCorner(cD.rows(), cD.cols()-1);

        v = Qx + D.transpose()*Qu;
        P = Qxx + D.transpose()*Qux;

        VectorXd tmp3(dV.size());
        tmp3 << c.transpose()*Qu, 0.5*c.transpose()*Quu*c;
        dV = dV + tmp3;

        vs.col(k) = v;
        Ps.at(k) = P;

        cs.col(k) = c;
        Ds.at(k) = D;
    }

    double s1 = 0.1;
    double s2 = 0.5;
    double b1 = 0.25;
    double b2 = 2.0;

    // measured change in V
    double eps = 2.2e-16;
    double dVm = eps;
    double Vn;

    // while loop to optimize step size (a) during forward pass
    while (dVm > 0)
    {
        // variation
        VectorXd dx(n);
        dx.setZero();

        // varied x
        VectorXd xn = in_struct.x0;

        // new measured cost
        Vn = 0;

        // begin forward pass
        for (int k = 0; k < N; k++)
        {
            VectorXd u = in_struct.us.col(k);

            VectorXd c = cs.col(k);
            MatrixXd D = Ds.at(k);

            VectorXd du = a*c + D*dx;
            VectorXd un = u + du;

            int temp = un.size();

            VectorXd umin = in_struct.S.getumin();
            VectorXd umax = in_struct.S.getumax();
            for (int i = 0; i < temp; i++)
            {
                if (un(i) < umin(i))
                {
                    un(i) = umin(i);
                    du(i) = un(i) - u(i);
                }
                else if (un(i) > umax(i))
                {
                    un(i) = umax(i);
                    du(i) = un(i) - u(i);
                }
                else {}
            }

            func_in LIn = {k, xn, un};
            LOut = in_struct.S.L(LIn);

            f_out fOut = in_struct.S.f(LIn);
            xn = fOut.x; // update xn

            dx = xn - xs.col(k+1);

            Vn = Vn + LOut.L;

            dus.col(k) = du;

        }

        // terminal cost update
        VectorXd dummyU(in_struct.us.col(0).size());
        dummyU.setZero();
        func_in LIn = {N, xn, dummyU};
        LOut = in_struct.S.L(LIn);

        Vn = Vn + LOut.L;

        dVm = Vn - V;

        if (dVm > 0)
        {
            a = b1*a;
            if (a < amin)
                break;

            continue;
        }

        VectorXd avec(2);
        avec << a, a*a;
        double dVp = avec.transpose()*dV;

        double r = dVm / dVp;

        if (r < s1)
            a = b1*a;
        else
        {
            if (r >= s2)
                a = b2*a;
        }
    }

    ddp_out ret = {dus, V, Vn, dV, a};
    return ret;
}
