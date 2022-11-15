#ifndef PUMPCONTROL_QP
#define PUMPCONTROL_QP

#include "PumpController_BasicFunctions.h"

#include <iostream>
#include <cmath>
#include <chrono>

#include "QP_BasicFiles/QuadProg++.hh"

#include "qpSWIFT_BasicFiles/include/timer.h"
#include "qpSWIFT_BasicFiles/include/GlobalOptions.h"
#include "qpSWIFT_BasicFiles/include/Prime.h"

#define     zero_threshold      1e-12

void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B);
void Matrix4QP_Addition(MatrixNd& A_base, VectorNd& B_base, MatrixNd A_new, VectorNd B_new);
void PumpPressureController_LinearMPC(int N, double dT, double Ps_now, VectorNd Ps_des_window, VectorNd Qact_des_window,
                                      VectorNd &X_MPC, VectorNd &U_MPC);

void PumpPressure_VirtualRefMotion(double t_now, double dT_Prev, VectorNd &Psdes_Future, VectorNd &Qdes_Future);
void PumpPressure_VirtualDynamics(double Ps_now, double wp_now, double Q_act, double dT_SIM, double& Ps_next);
void PumpFlowControl_Active_LIGHT2(double Ps_now, double Ps_ref, double dPs_ref, double &u_out);
void PumpFlowControl_Active_QuadTest(double Ps_now, char PsRef_type, double &u_out);

enum SolverTypes {
    SolverIsQuadProg = 0,
    SolverIsQPSwift = 1,
};

class PumpControl_QP // Using QuadProg++
{

private:
    int Solver;

public:
    int n_X, n_U, n_var;

public:
    MatrixNd A_inequ,B_inequ;
    MatrixNd A_equ,B_equ;
    MatrixNd A_cost,B_cost;
    MatrixNd P_cost,Q_cost;

public:
    PumpControl_QP(int SolverSelection = SolverIsQPSwift)
    {
        Solver = SolverSelection;
    }

public:
    void SelectSolver(int _solver) {Solver = _solver;}
    int WhichSolver() {return Solver;}

    ///// QuadProg's Variables /////////////////////////////////////////////////////////////////////////////
public:
    int NUMCOLS;
    int NUMCOST;
    int NUMEQ;
    int NUMINEQ;

public:

    MatrixNd X;

    void setNums(int _xlength, int numCost, int numEqConstraints, int numIneqConstraints)
    {
        NUMCOLS = _xlength;
        NUMCOST = numCost;
        NUMEQ = numEqConstraints;
        NUMINEQ = numIneqConstraints;

        //maybe matrix assignment here
        X.resize(_xlength,1);
        X = MatrixNd::Zero(NUMCOLS,1);
        A_equ.resize(NUMEQ, NUMCOLS);
        A_equ = MatrixNd::Zero(NUMEQ, NUMCOLS);
        B_equ.resize(NUMEQ, 1);
        B_equ = MatrixNd::Zero(NUMEQ, 1);

        A_inequ.resize(NUMINEQ, NUMCOLS);
        A_inequ = MatrixNd::Zero(NUMINEQ, NUMCOLS);
        B_inequ.resize(NUMINEQ, 1);
        B_inequ = MatrixNd::Zero(NUMINEQ, 1);

        P_cost.resize(NUMCOLS,NUMCOLS);
        P_cost = MatrixNd::Zero(NUMCOLS,NUMCOLS);
        Q_cost.resize(NUMCOLS,1);
        Q_cost = MatrixNd::Zero(NUMCOLS,1);
    }

    void make_EQ(MatrixNd A, MatrixNd b)
    {
        //Aeq*X = Beq
        A_equ = A;
        B_equ = b;
    }

    void update_Bequ(MatrixNd b)
    {
        B_equ = b;
    }

    void make_IEQ(MatrixNd A, MatrixNd b)
    {
        // Aineq*X < Bineq
        A_inequ = A;
        B_inequ = b;
    }

    void update_Binequ(MatrixNd b)
    {
        B_inequ = b;
    }

    void make_COST(MatrixNd A, MatrixNd b)
    {
        //min (0.5* x P x + Q' x) << (0.5*x(AT*A)x - (AT*b)*x + bT*b) << (0.5*|Ax-b|^2)
        A_cost = A; // m-by-n matrix
        B_cost = b;
        int m = A_cost.transpose().cols();
        int n = A_cost.transpose().rows();
        for(int i=0;i<n;i++) {
            P_cost.block(i,0,1,n) = A_cost.transpose().block(i,0,1,m) * A_cost;
        }
        Q_cost = -A_cost.transpose() * B_cost;

    }

    void update_Bcost(MatrixNd b)
    {
        B_cost = b;
        Q_cost = -A_cost.transpose() * B_cost;
    }

    void make_COST_direct(MatrixNd _P, MatrixNd _Q)
    {
        //min (0.5* x P x + Q'x)
        P_cost = _P;
        Q_cost = _Q;
    }

    MatrixNd solve_QP()
    {
        quadprogpp::Vector<double> outX;
        quadprogpp::Matrix<double> G;
        quadprogpp::Vector<double> g0;
        quadprogpp::Matrix<double> CE;
        quadprogpp::Vector<double> ce0;
        quadprogpp::Matrix<double> CI;
        quadprogpp::Vector<double> ci0;
        //min 0.5 * x G x + g0 x
        //CE^T x + ce0 = 0
        //CI^T x + ci0 >= 0

        G.resize(NUMCOLS,NUMCOLS);
        g0.resize(NUMCOLS);
        for(int i=0;i<NUMCOLS;i++)
        {
            for(int j=0;j<NUMCOLS;j++)
            {
                G[i][j] = P_cost(i,j);
            }
            g0[i] = Q_cost(i,0);
        }
        CE.resize(NUMCOLS,NUMEQ);
        ce0.resize(NUMEQ);
        for(int i=0;i<NUMCOLS;i++)
        {
            for(int j=0;j<NUMEQ;j++)
            {
                CE[i][j] = -A_equ(j,i);
            }
        }
        for(int j=0;j<NUMEQ;j++)
        {
            ce0[j] = B_equ(j,0);
        }
        CI.resize(NUMCOLS,NUMINEQ);
        ci0.resize(NUMINEQ);
        for(int i=0;i<NUMCOLS;i++)
        {
            for(int j=0;j<NUMINEQ;j++)
            {
                CI[i][j] = -A_inequ(j,i);
            }
        }
        for(int j=0;j<NUMINEQ;j++)
        {
            ci0[j] = B_inequ(j,0);
        }
        outX.resize(NUMCOLS);

        solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
        for(int i=0;i<NUMCOLS;i++)
        {
            X(i,0) = outX[i];
        }

        return X;

    }

    ///// qpSWIFT's Variables ////////////////////////////////////////////////////////////////////////////////

private:
    qp_int  n_var_QPswift;
    qp_int  n_cost_QPswift;
    qp_int  n_equ_QPswift;
    qp_int  n_inequ_QPswift;

    qp_int  exitflag_QPswift;

public:
    qp_int GetNumberOfNonZero_QPswift(MatrixNd mat)
    {
        int n_cols = mat.cols();
        int n_rows = mat.rows();

        qp_int nnz = 0; // the number of non-zero components
        for(int i=0; i<n_cols; i++)
        {
            for(int j=0; j<n_rows; j++)
            {
                if(fabs(mat(j,i)) > zero_threshold)
                {
                    nnz++;
                }
            }
        }
        return nnz;
    }

    void ConvertMatrixA_Full2CCS_QPswift(MatrixNd A, qp_real* A_x, qp_int* A_i, qp_int* A_p)
    {
        // ref : https://ko.wikipedia.org/wiki/%ED%9D%AC%EC%86%8C%ED%96%89%EB%A0%AC

        // The real array A_x holds all the nonzero entries of A in column major format
        // The integer array A_i holds the rows indices of the corresponding elements in A_x
        // The integer array A_p is defined as
        //   – A_p[0] = 0
        //   – A_p[i] = A_p[i − 1] + number of non-zeros in i th column of A

        //    For the following sample matrix A,
        //    A =  [ 4.5 0   3.2 0
        //           3.1 2.9 0   2.9
        //           0   1.7 3.0 0
        //           3.5 0.4 0   1.0 ]

        //    we have the following CCS representation
        //    >> double A_x ={4.5, 3.1, 3.5, 2.9, 1.7, 0.4, 3.2, 3.0, 2.9, 1.0}
        //    >> int A_i ={0, 1, 3, 1, 2, 3, 0, 2, 1, 3}
        //    >> int A_p ={0, 3, 6, 8, 10}

        int n_cols = A.cols();
        int n_rows = A.rows();

        int cnt_A_x = 0;
        int cnt_A_i = 0;
        int cnt_A_p = 0;
        A_p[0] = 0;

        for(int i=0; i<n_cols; i++)
        {
            for(int j=0; j<n_rows; j++)
            {
                if(fabs(A(j,i)) > zero_threshold)
                {
                    // Get A_x
                    A_x[cnt_A_x] = A(j,i);
                    cnt_A_x++;

                    // Get A_i
                    A_i[cnt_A_i] = j;
                    cnt_A_i++;

                    // Get A_p
                    cnt_A_p++;
                }
            }
            A_p[i+1] = cnt_A_p;
        }
    }

    void ConvertMatrixP_Full2CCS_QPswift(MatrixNd P, qp_real* P_x, qp_int* P_i, qp_int* P_p)
    {
        // Convert Symmetric Matrix P >> Upper Triangular Matrix
        // and Make it CCS format

        int n_cols = P.cols();

        int cnt_P_x = 0;
        int cnt_P_i = 0;
        int cnt_P_p = 0;
        P_p[0] = 0;

        for(int i=0; i<n_cols; i++)
        {
            for(int j=0; j<i+1; j++)
            {
                if(fabs(P(j,i)) > zero_threshold)
                {
                    // Get P_x
                    P_x[cnt_P_x] = P(j,i);
                    cnt_P_x++;

                    // Get P_i
                    P_i[cnt_P_i] = j;
                    cnt_P_i++;

                    // Get P_p
                    cnt_P_p++;
                }
            }
            P_p[i+1] = cnt_P_p;
        }
    }


    void ConvertVector2Array_QPswift(VectorNd vec, qp_int n, qp_real* v)
    {
        for(int i=0;i<n;i++) {
            v[i] = vec(i);
        }
    }

    void ConvertArray2Vector_QPswift(qp_real* v, qp_int n, VectorNd &vec)
    {
        for(int i=0;i<n;i++) {
            vec(i) = v[i];
        }
    }

    void Example_Problem()
    {
        ///////////////////////////////////
        /// Problem

        int n_var = 2;
        int n_cost = 2;
        int n_inequ = 0;
        int n_equ = 1;

        MatrixNd A_cost(2,2);
        MatrixNd B_cost(2,1);
        A_cost << 4.0,1.0,
                  1.0,2.0;
        B_cost << 1.0,
                  1.0;

        MatrixNd A_inequ(n_inequ,n_var);
        MatrixNd Bl_inequ(n_inequ,1);
        MatrixNd Bu_inequ(n_inequ,1);
    //        A_inequ <<  1.0, 0.0,
    //                    0.0, 1.0;
    //        Bl_inequ << 0.0, 0.0;
    //        Bu_inequ << 0.7, 0.7;

        MatrixNd A_equ(n_equ,n_var);
        MatrixNd B_equ(n_equ,1);
        A_equ << 1.0, 1.0;
        B_equ << 1.0;

        VectorNd sol = VectorNd::Zero(n_var);
        /// Answer : 0.25 0.75
        ///////////////////////////////////////
        system_clock::time_point StartTime_QPSwift = system_clock::now();

        qp_int     P_nnz,A_nnz,G_nnz;
        qp_real  *P_x;
        qp_int     *P_i,*P_p;
        qp_real  *q_x;
        qp_real  *A_x;
        qp_int     *A_i,*A_p;
        qp_real  *b_x;
        qp_real  *G_x;
        qp_int     *G_i,*G_p;
        qp_real  *h_x;

        // 1. Define the cost functions.
        MatrixNd P = A_cost;
        VectorNd q = B_cost;
        P_nnz = (qp_int)this->GetNumberOfNonZero_QPswift(P);
        P_x = (qp_real*)malloc(P_nnz*sizeof(qp_real));
        P_i = (qp_int*)malloc(P_nnz*sizeof(qp_int));
        P_p = (qp_int*)malloc((n_var+1)*sizeof(qp_int));
        this->ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
        q_x = (qp_real*)malloc(n_var*sizeof(qp_real));
        this->ConvertVector2Array_QPswift(q,n_var,q_x);

        // 2. Define the equality constraints.
        MatrixNd A = A_equ;
        VectorNd b = B_equ;
        A_nnz = (qp_int)this->GetNumberOfNonZero_QPswift(A);
        A_x = (qp_real*)malloc(A_nnz*sizeof(qp_real));
        A_i = (qp_int*)malloc(A_nnz*sizeof(qp_int));
        A_p = (qp_int*)malloc((n_var+1)*sizeof(qp_int));
        this->ConvertMatrixA_Full2CCS_QPswift(A,A_x,A_i,A_p);
        b_x = (qp_real*)malloc(n_equ*sizeof(qp_real));
        this->ConvertVector2Array_QPswift(b,n_equ,b_x);

        // 3. Define the inequality constraints.
        MatrixNd G = MatrixNd::Zero(2*n_inequ,n_var);
        MatrixNd h = MatrixNd::Zero(2*n_inequ,1);
        G.block(0,0,n_inequ,n_var) = -A_inequ;
        G.block(n_inequ,0,n_inequ,n_var) = A_inequ;
        h.block(0,0,n_inequ,1) = -Bl_inequ;
        h.block(n_inequ,0,n_inequ,1) = Bu_inequ;
        G_nnz = (qp_int)this->GetNumberOfNonZero_QPswift(G);
        G_x = (qp_real*)malloc(G_nnz*sizeof(qp_real));
        G_i = (qp_int*)malloc(G_nnz*sizeof(qp_int));
        G_p = (qp_int*)malloc((n_var+1)*sizeof(qp_int));
        this->ConvertMatrixA_Full2CCS_QPswift(G,G_x,G_i,G_p);
        h_x = (qp_real*)malloc((2*n_inequ)*sizeof(qp_real));
        this->ConvertVector2Array_QPswift(h,(2*n_inequ),h_x);

    //        QPswift *myQP;
    //        myQP = QP_SETUP(n_var,n_inequ,n_equ, P_p,P_i,P_x, A_p,A_i,A_x, G_p,G_i,G_x,q_x,h_x,b_x, 0.0,nullptr);

    //        // 5. Solve.
    //        int ExitCode = QP_SOLVE(myQP);

        QPswift *myQP = QP_SETUP_NoInequ(n_var,n_equ,
                                         P_p,P_i,P_x,
                                         A_p,A_i,A_x,
                                         q_x,b_x,
                                         0.0,nullptr);
        QP_SOLVE_NoInequ(myQP);
        this->ConvertArray2Vector_QPswift(myQP->x, n_var, sol);

        QP_CLEANUP(myQP);
        free(P_x);  free(P_i);  free(P_p);
        free(A_x);  free(A_i);  free(A_p);
        free(G_x);  free(G_i);  free(G_p);
        free(q_x);  free(b_x);  free(h_x);

        system_clock::time_point EndTime_QPSwift = system_clock::now();
        FILE_LOG(logINFO) << "qpSWIFT Solution : " << sol.transpose();

        ///////////////////////////////////////
        system_clock::time_point StartTime_QuadProg = system_clock::now();

        // 1. Define the number of Variables.
        this->setNums(n_var,n_cost,n_equ,2*n_inequ);

        // 2. Define the cost functions.
        this->make_COST_direct(A_cost, B_cost);

        // 3. Define the equality constraints.
        this->make_EQ(A_equ, B_equ);

        // 4. Define the inequality constraints.
        MatrixNd Ai = MatrixNd::Zero(2*n_inequ,n_var);
        MatrixNd Bi = MatrixNd::Zero(2*n_inequ,1);
        Ai.block(0,0,n_inequ,n_var) = -A_inequ;
        Ai.block(n_inequ,0,n_inequ,n_var) = A_inequ;
        Bi.block(0,0,n_inequ,1) = -Bl_inequ;
        Bi.block(n_inequ,0,n_inequ,1) = Bu_inequ;
        this->make_IEQ(Ai,Bi);

        // 5. Solve.
        sol = this->solve_QP();
        system_clock::time_point EndTime_QuadProg = system_clock::now();
        FILE_LOG(logINFO) << "QuadProg Solution : " << sol.transpose();

        //////////////////////////////////////////////////////////////////////////
    //        microseconds t_calc_OSQP = duration_cast<std::chrono::microseconds>(EndTime_OSQP - StartTime_OSQP);
        microseconds t_calc_QPSwift = duration_cast<std::chrono::microseconds>(EndTime_QPSwift - StartTime_QPSwift);
        microseconds t_calc_QuadProg = duration_cast<std::chrono::microseconds>(EndTime_QuadProg - StartTime_QuadProg);
    //        FILE_LOG(logDEBUG) << "Time OSQP : "<< t_calc_OSQP.count() <<" usecond(s).";
        FILE_LOG(logDEBUG) << "Time QPSwift : "<< t_calc_QPSwift.count() <<" usecond(s).";
        FILE_LOG(logDEBUG) << "Time QuadProg : "<< t_calc_QuadProg.count() <<" usecond(s).";

    }
};


#endif // PUMPCONTROL_QP

