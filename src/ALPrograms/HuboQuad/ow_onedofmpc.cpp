#include "ow_onedofmpc.h"
ow_onedofMPC::ow_onedofMPC(int _N, double _dt, double _maxu)
{
    N = _N;
    dt = _dt;
    flyT = 0.1;//will be set later
    maxu = _maxu;

    H = MatrixNd::Zero(N,N);
    F = MatrixNd::Zero(N,1);

    A_eq = MatrixNd::Zero(0,N);
    B_eq = MatrixNd::Zero(0,1);
    A_ineq = MatrixNd::Zero(2*N,N);//max,min
    B_ineq = MatrixNd::Zero(2*N,1);

    A = MatrixNd::Zero(2,2);
    B = MatrixNd::Zero(2,1);
    C = MatrixNd::Zero(2,2);
    D = MatrixNd::Zero(2,1);
    ZD = MatrixNd::Zero(2,1);
    Afly = MatrixNd::Zero(2,2);

    A(0,0) = 1;
    A(0,1) = dt;
    A(1,0) = 0;
    A(1,1) = 1;
    An[0] = MatrixNd::Zero(2,2);
    An[0](0,0) = 1;
    An[0](1,1) = 1;
    for(int i=1;i<N;i++)
    {
        An[i] = An[i-1]*A;
    }

    B(0,0) = dt*dt*0.5;
    B(1,0) = dt;

    C(0,0) = 1;
    C(0,1) = 0;
    C(1,0) = 0;
    C(1,1) = 1;

    D(0,0) = 0;
    D(1,0) = 0;

    Afly(0,0) = 1;
    Afly(0,1) = flyT;
    Afly(1,0) = 0;
    Afly(1,1) = 1;

    R = MatrixNd::Zero(N,N);
    Q = MatrixNd::Zero(2*N,2*N);

    V = MatrixNd::Zero(2*N,N);
    v = MatrixNd::Zero(2*N,2);
    ref_now = MatrixNd::Zero(2*N,1);
    xnow = MatrixNd::Zero(2,1);

    //set min-max
    if(maxu<0.1) {maxu = 0.1;}//bigger then zero
    for(int i=0;i<N;i++)
    {
        A_ineq(i*2,i) = 1;
        B_ineq(i*2,0) = maxu;
        A_ineq(i*2+1,i) = -1;
        B_ineq(i*2+1,0) = maxu;
    }

}


double ow_onedofMPC::getddx(vec3 x0, std::vector<vec3> ref, int stanceN, double _flyT, double _R, double _Qx, double _Qv, double _maxu)
{

    flyT = _flyT;
    Afly(0,1) = flyT;
    maxu = _maxu;
    //set min-max
    if(maxu<0.1) {maxu = 0.1;}//bigger then zero
    for(int i=0;i<N;i++)
    {
        A_ineq(i*2,i) = 1;
        B_ineq(i*2,0) = maxu;
        A_ineq(i*2+1,i) = -1;
        B_ineq(i*2+1,0) = maxu;
    }

    for(int i=0;i<N;i++)
    {
        R(i,i) = _R;
        Q(i*2,i*2) = _Qx;
        Q(i*2+1,i*2+1) = _Qv;
        int ii = min(i,ref.size()-1);
        if(ii>=0)
        {
            ref_now(i*2,0) = ref[ii][0];
            ref_now(i*2+1,0) = ref[ii][1];
        }
        else
        {
            ref_now(i*2,0) = 0;
            ref_now(i*2+1,0) = 0;
        }
    }
    xnow(0,0) = x0[0];
    xnow(1,0) = x0[1];


    //make V and v here~~~
    //if 1 step remaining on this step,
    //stanceN = =1
    //then, apply once
    for(int i=0;i<N;i++)
    {
        //std::cout<<"t1 "<<"i "<<i<<v<<std::endl;
        if(i<stanceN)
        {
            v.block(i*2,0,2,2) = C*An[i];
        }
        else
        {
            v.block(i*2,0,2,2) = C*An[i-stanceN]*Afly*An[stanceN];
        }

        for(int j=0;j<N;j++)
        {
            //std::cout<<"t2 "<<"i "<<i<<"j "<<j<<std::endl;
            if(i>j)
            {
                if((i-j)<stanceN)
                {
                    V.block(i*2,j,2,1) = C*An[i-j]*B;
                }
                else
                {
                    V.block(i*2,j,2,1) = C*An[i-j-stanceN]*Afly*An[stanceN]*B;
                }
            }
            else if(i==j)
            {
                V.block(i*2,j,2,1) = D;
            }
            else
            {
                V.block(i*2,j,2,1) = ZD;
            }
        }
    }
//std::cout<<"t3"<<std::endl;

    //~~~~

    H = V.transpose()*Q*V + R;
    F = -V.transpose()*Q*ref_now + V.transpose()*Q*v*xnow;

    int NUMCOLS = A.cols();
    int NUMEQ = A_eq.rows();
    int NUMINEQ = A_ineq.rows();

    quadprogpp::Vector<double> outX;
    quadprogpp::Matrix<double> G;
    quadprogpp::Vector<double> g0;
    quadprogpp::Matrix<double> CE;
    quadprogpp::Vector<double> ce0;
    quadprogpp::Matrix<double> CI;
    quadprogpp::Vector<double> ci0;
    //CE^T x + ce0 = 0
    //CI^T x + ci0 >= 0

    G.resize(NUMCOLS,NUMCOLS);
    g0.resize(NUMCOLS);
    for(int i=0;i<NUMCOLS;i++)
    {
        for(int j=0;j<NUMCOLS;j++)
        {
            G[i][j] = H(i,j);
        }
        g0[i] = F(i,0);
    }
    CE.resize(NUMCOLS,NUMEQ);
    ce0.resize(NUMEQ);
    for(int i=0;i<NUMCOLS;i++)
    {
        for(int j=0;j<NUMEQ;j++)
        {
            CE[i][j] = -A_eq(j,i);
        }
    }
    for(int j=0;j<NUMEQ;j++)
    {
        ce0[j] = B_eq(j,0);
    }
    CI.resize(NUMCOLS,NUMINEQ);
    ci0.resize(NUMINEQ);
    for(int i=0;i<NUMCOLS;i++)
    {
        for(int j=0;j<NUMINEQ;j++)
        {
            CI[i][j] = -A_ineq(j,i);
        }
    }
    for(int j=0;j<NUMINEQ;j++)
    {
        ci0[j] = B_ineq(j,0);
    }
    outX.resize(NUMCOLS);

    solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
    return outX[0];
}
