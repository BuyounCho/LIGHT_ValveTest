#ifndef OW_CPLEX_H
#define OW_CPLEX_H
//solver run in c
//#define IL_STD
//#include <ilcplex/ilocplex.h>
//#include <ilcplex/cplex.h>
#include "QuadProg++.hh"
#include "rbdl/rbdl.h"


//actually not using cplex now

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
class OW_CPLEX
{
private:
    int NUMCOLS;//length of X
    int NUMEQ;
    int NUMINEQ;
    int NUMROWS;//# of constraints
    int NUMNZ;//# of nonzero values in constraints
    int NUMQNZ;//# of nonzero values in Quadratic Matrix
//    int solstat;
//    double   objval;
//    const static int objsen = CPX_MIN;//minimize problem
    double u;//friction coef
    double minfz;//originally 2
    double maxT;

    MatrixNd A_ineq,B_ineq;
    MatrixNd A_eq,B_eq;
    MatrixNd A_eqineq,B_eqineq;
public:
    OW_CPLEX()
    {
        u = 0.8;
        minfz = 30;
        maxT = 250;
    }
    MatrixNd H,F;
    VectorNd X;
public:
    void setNums(int _xlength, int numEqConstraints, int numIneqConstraints)
    {
        NUMCOLS = _xlength;//should be 18+12+3*contact
        NUMEQ = numEqConstraints;//NUMCOLS(dynamics) + NUMCOLS(contactconstraints + swingleg)
        //contact to be weight?
        NUMINEQ = numIneqConstraints+12*2;//friction cone approximation -ufz<fx<ufz, -ufz<fy<ufz ->4
        NUMROWS = NUMEQ + NUMINEQ;
        NUMNZ = NUMCOLS*NUMROWS;//maybe will be automatically generated
        NUMQNZ = NUMCOLS*NUMCOLS;//
        //maybe matrix assignment here
        X = VectorNd::Zero(NUMCOLS);
        A_eq = MatrixNd::Zero(NUMEQ, NUMCOLS);
        B_eq = MatrixNd::Zero(NUMEQ, 1);

        A_ineq = MatrixNd::Zero(NUMINEQ, NUMCOLS);
        B_ineq = MatrixNd::Zero(NUMINEQ, 1);

        A_eqineq = MatrixNd::Zero(NUMROWS, NUMCOLS);
        B_eqineq = MatrixNd::Zero(NUMROWS, 1);

        H = MatrixNd::Zero(NUMCOLS,NUMCOLS);
        F = MatrixNd::Zero(NUMCOLS,1);

    }    
    double t1,t2;
    void MakeEqFly(MatrixNd _M, MatrixNd _nonlin)
    {
        //eom[M -I][ddq;t] = nonlin
        A_eq.block(0,0,_M.rows(),_M.cols()) = _M;//ddq
        A_eq.block(6,18,12,12) = -MatrixNd::Identity(12,12);//Tau
        B_eq.block(0,0,_nonlin.rows(),_nonlin.cols()) = -_nonlin;//nonlin

        A_eqineq.block(0,0,NUMEQ,NUMCOLS) = A_eq;
        B_eqineq.block(0,0,NUMEQ,1) = B_eq;

    }
    void MakeEq(MatrixNd _M, MatrixNd _nonlin, MatrixNd _J)
    {
        //eom[M -I -JT]        
        A_eq.block(0,0,_M.rows(),_M.cols()) = _M;//ddq
        A_eq.block(6,18,12,12) = -MatrixNd::Identity(12,12);//Tau
        A_eq.block(0,18+12,_J.cols(),_J.rows()) = -_J.transpose();//force
        B_eq.block(0,0,_nonlin.rows(),_nonlin.cols()) = -_nonlin;//nonlin


        A_eqineq.block(0,0,NUMEQ,NUMCOLS) = A_eq;
        B_eqineq.block(0,0,NUMEQ,1) = B_eq;

    }
    void MakeIneq()
    {
        int ineqcnt = 0;
        for(int i=18+12;i<NUMCOLS;i = i+3)
        {
            A_ineq(ineqcnt,i)   =  1;
            A_ineq(ineqcnt,i+2) = -1*u/sqrt(2.0);//fx<ufz/sqrt(2)
            ineqcnt++;
            A_ineq(ineqcnt,i)   = -1;
            A_ineq(ineqcnt,i+2) = -1*u/sqrt(2.0);//-ufz/sqrt(2)<fx
            ineqcnt++;
            A_ineq(ineqcnt,i+1) =  1;
            A_ineq(ineqcnt,i+2) = -1*u/sqrt(2.0);//fy<ufz/sqrt(2)
            ineqcnt++;
            A_ineq(ineqcnt,i+1) = -1;
            A_ineq(ineqcnt,i+2) = -1*u/sqrt(2.0);//-ufz/sqrt(2)<fy
            ineqcnt++;
            A_ineq(ineqcnt,i+2) = -1;//fz>minfz
            B_ineq(ineqcnt,0) = -minfz;
            ineqcnt++;
        }
        //torque minmax
        for(int i=0;i<12;i++)
        {
            A_ineq(ineqcnt,18+i) = 1;//T<Tmax
            B_ineq(ineqcnt,0) = maxT;
            ineqcnt++;
            A_ineq(ineqcnt,18+i) = -1;//-T<Tmax
            B_ineq(ineqcnt,0) = maxT;
            ineqcnt++;
        }
        A_eqineq.block(NUMEQ,0,NUMINEQ,NUMCOLS) = A_ineq;
        B_eqineq.block(NUMEQ,0,NUMINEQ,1) = B_ineq;
        //contact force friction cone
    }
    void MakeHF(std::vector<MatrixNd> As, std::vector<MatrixNd> Bs, std::vector<double> ws)
    {        
        int nn = As.size();
        H = MatrixNd::Zero(NUMCOLS,NUMCOLS);
        F = MatrixNd::Zero(NUMCOLS,1);
        for(int i=0;i<nn;i++)
        {            
            H = H + ws[i]*ws[i]*(As[i].transpose()*As[i]);
            F = F - ws[i]*ws[i]*(As[i].transpose()*Bs[i]);
        }
    }
    VectorNd calcX4(MatrixNd A, MatrixNd B)
    {
        Eigen::JacobiSVD<MatrixNd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        return svd.solve(B);

    }
    VectorNd calcX3(MatrixNd A, MatrixNd B,MatrixNd Aeq, MatrixNd Beq)
    {
        H = A.transpose()*A;
        H = H+MatrixNd::Identity(H.cols(),H.rows())*1e-10;//for strictly converging
        F = -A.transpose()*B;
        NUMCOLS = A.cols();
        NUMEQ = Aeq.rows();
        NUMINEQ = 0;
        X = VectorNd::Zero(NUMCOLS);
        A_eq = Aeq;
        B_eq = Beq;

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
        outX.resize(NUMCOLS);
        solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
        for(int i=0;i<NUMCOLS;i++)
        {
            X[i] = outX[i];
        }

        return X;
    }
    VectorNd calcX2()
    {
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
        for(int i=0;i<NUMCOLS;i++)
        {
            X[i] = outX[i];
        }

        return X;

    }
    VectorNd calcX()
    {
//        int NUMCOLS;//length of X
//        int NUMROWS;//# of constraints
//        int NUMNZ;//# of nonzero values in constraints
//        int NUMQNZ;//# of nonzero values in Quadratic Matrix
//           t1 = rt_timer_read();
//           double*    zobj = new double[NUMCOLS];//linear objective
//           double*    zrhs = new double[NUMROWS];//right hand side of constraint
//           char*    zsense = new char[NUMROWS];//type of constraint 'L','E','G'
//           int*     zmatbeg = new int[NUMCOLS];//sum of zmatcnt lower then i
//           int*     zmatcnt = new int[NUMCOLS];//# of nonzero values in A related to each x (Ax~B)
//           int*     zmatind = new int[NUMNZ];//indecator of what constraint is that in value
//           double*     zmatval = new double[NUMNZ];//constraint matrix value
//           double*     zlb = new double[NUMCOLS];//x lowerbound
//           double*     zub = new double[NUMCOLS];//x upperbound
//           int*     zqmatbeg = new int[NUMCOLS];//sum of zqmatcnt lower then i
//           int*     zqmatcnt = new int[NUMCOLS];//# of nonzero values in H related to each x
//           int*     zqmatind = new int[NUMQNZ];//indecator of what coloum is that in value
//           double*     zqmatval = new double[NUMQNZ];//quadratic objective value

//           double vals[NUMCOLS];//optimized X
//           double pi[NUMROWS];
//           double slack[NUMROWS];
//           double dj[NUMCOLS];

//           //if too slow, needs nonzero and zero
//           for(int i=0;i<18;i++)//acc limit
//           {
//               zlb[i] = -150.0;//
//               zub[i] = 150.0;//
//           }
//           for(int i=18;i<18+12;i++)//torque limit
//           {
//               zlb[i] = -500.0;//
//               zub[i] = 500.0;//
//           }
//           for(int i=18+12;i<NUMCOLS;i++)//contact force limit
//           {
//               zlb[i] = -1000.0;//min about -100kg
//               if((i-(18+12))%3==2) zlb[i] = minfz;//at least 0 in z direction(or little bit more?)
//               zub[i] = 1000.0;//max about 100 kg
//           }

//           for(int i=0;i<NUMCOLS;i++)
//           {
//               double Fni = F(i,0);
//               zobj[i] = Fni;//F

//               zqmatbeg[i] = i*(NUMCOLS);//H_n, xpart
//               zqmatcnt[i] = NUMCOLS;

//               for(int j=0;j<NUMCOLS;j++)
//               {
//                   double Hnij= H(i,j);//*0.5? don't know
//                   zqmatind[i*(NUMCOLS) + j] = j;  //H
//                   zqmatval[i*(NUMCOLS) + j] = Hnij;//H
//               }
//           }
//           //wrong frome here?
//           for(int i=0;i<NUMROWS;i++)
//           {
//               if(i<NUMEQ)
//               {
//                    zsense[i] = 'E';//AX=B
//               }
//               else
//               {
//                    zsense[i] = 'L';//AX<B
//               }
//               zrhs[i] = B_eqineq(i,0);//b0
//           }
//           for(int i=0;i< NUMCOLS;i++)
//           {
//               zmatbeg[i] = i*NUMROWS;
//               zmatcnt[i] = NUMROWS;

//               for(int j=0;j<NUMROWS;j++)
//               {
//                   double Acij = (A_eqineq(j,i));
//                   zmatind[i*NUMROWS + j] = j;//Ac, x
//                   zmatval[i*NUMROWS + j] = Acij;
//               }
//           }


//           int status;
//           CPXENVptr env = NULL;
//           CPXLPptr lp = NULL;
//           env = CPXopenCPLEX(&status);
//           if ( env == NULL ) {
//           char  errmsg[CPXMESSAGEBUFSIZE];
//              printf (  "Could not open CPLEX environment.\n");
//              CPXgeterrorstring (env, status, errmsg);
//              printf (  "%s", errmsg);
//           }
////           status = CPXsetintparam (env, CPX_PARAM_QPMETHOD, CPX_ALG_DUAL);
////             if ( status ) {
////                printf (
////                         "Failure to set algorithm, error %d.\n", status);
////             }
//           status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);///////////////////
//             if ( status ) {
//                printf (
//                         "Failure to turn on screen indicator, error %d.\n", status);
//             }
////           status = CPXsetdblparam(env,CPX_PARAM_TILIM,0.004);//calculate less then 4ms
////           if ( status ) {
////              printf (
////                       "Failure to set time limit, error %d.\n", status);
////           }


//            status = CPXsetintparam (env, CPX_PARAM_THREADS, 1);
//              if ( status ) {
//                  printf (
//                           "Failure to set to singlethread, error %d.\n", status);
//               }
//          char probname[16] = "QP_cplex";
//          int numcols = NUMCOLS;
//          int numrows = NUMROWS;
//          //numrows = 0;//dynamics only
//          lp = CPXcreateprob (env, &status, probname);
//          if ( lp == NULL ) {
//             printf (  "Failed to create problem.\n");
//          }
//          status = CPXcopylp (env, lp, numcols, numrows, objsen, zobj, zrhs,
//                              zsense, zmatbeg, zmatcnt, zmatind, zmatval,
//                              zlb, zub, NULL);
//          if ( status ) {
//             printf (  "Failed to copy problem data.\n");
//          }

//          status = CPXcopyquad (env, lp, zqmatbeg, zqmatcnt, zqmatind, zqmatval);
//          if ( status ) {
//             printf (  "Failed to copy quadratic matrix.\n");
//          }
//           status = CPXqpopt (env, lp);
//           if ( status ) {
//              printf (  "Failed to optimize QP.\n");
//              exit(0);

//           }

//           status = CPXsolution (env, lp, &solstat, &objval, vals, pi, slack, dj);
//           if ( status ) {
//              printf (  "Failed to obtain solution.\n");
//           }
//           else
//           {
//                for(int i=0;i<NUMCOLS;i++)
//                {
//                    X[i] = vals[i];
//                }
//           }

//           if ( lp != NULL ) {
//                status = CPXfreeprob (env, &lp);
//                if ( status ) {
//                   printf (  "CPXfreeprob failed, error code %d.\n", status);
//                }
//             }
//             if ( env != NULL ) {
//                status = CPXcloseCPLEX (&env);
//                if ( status ) {
//                   char  errmsg[CPXMESSAGEBUFSIZE];
//                   printf (  "Could not close CPLEX environment.\n");
//                   CPXgeterrorstring (env, status, errmsg);
//                   printf (  "%s", errmsg);
//                }
//             }


//             delete[]    zobj;
//             delete[]    zrhs;
//             delete[]    zsense;
//             delete[]     zmatbeg;
//             delete[]     zmatcnt;
//             delete[]     zmatind;
//             delete[]     zmatval;
//             delete[]     zlb;
//             delete[]     zub;
//             delete[]     zqmatbeg;
//             delete[]     zqmatcnt;
//             delete[]     zqmatind;
//             delete[]     zqmatval;

//           t2 = rt_timer_read();/////////////t5-t4, max0.002ms
           return X;

    }




public:

};

#endif // OW_CPLEX

