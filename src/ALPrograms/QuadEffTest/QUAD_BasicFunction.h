#ifndef QUAD_BASICFUNCTIONS_H
#define QUAD_BASICFUNCTIONS_H

#include <iomanip>
#include <iostream>
#include <cmath>

#include <chrono>
using namespace std;
using namespace std::chrono;

#include "RBLog.h"
#include "RBSharedMemory.h"
#include "JointInformation.h"
#include "BasicFiles/BasicJoint.h"

#include "rbdl/rbdl.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#include "QP_BasicFiles/QuadProg++.hh"

#include "qpSWIFT_BasicFiles/include/timer.h"
#include "qpSWIFT_BasicFiles/include/GlobalOptions.h"
#include "qpSWIFT_BasicFiles/include/Prime.h"

extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern JointControlClass       *joint;
extern JointControlClass       *jCon;

#define    SYS_DT  0.004
#define    SYS_FREQ  250.0

#define     g_const         9.81
#define     PI              3.141592
#define     D2R             PI/180.0
#define     R2D             180.0/PI

#define SAVEKIND            30
#define SAVENUM             100000

extern bool             save_Flag;
extern unsigned int     save_Index;
extern float            save_Buf[SAVEKIND][SAVENUM];
void                    save_PutData(unsigned int cur_Index);
void                    save_File(char* filecomment = "");


extern Vector3d zv;
extern Matrix3d I3;

void PrintHere(int n);
VectorNd Vector_LPF_2nd(VectorNd voo, VectorNd vo, VectorNd v_new, double f_cut);

void PolyTrajectory_1st(double tf, double tn,
                    double x, double xf,
                    double &xn, double &dxn, double &ddxn);
void PolyTrajectory_3rd(double tf, double tn,
                    double x, double dx, double xf, double dxf,
                    double &xn, double &dxn, double &ddxn);
void PolyTrajectory_5th(double tf, double tn,
                    double x, double dx, double ddx, double xf, double dxf, double ddxf,
                    double &xn, double &dxn, double &ddxn);

void PolyTrajectory_Vector_1st(double tf, double tn,
                          Vector3d X, Vector3d Xf,
                          Vector3d &Xn, Vector3d &dXn, Vector3d &ddXn);
void PolyTrajectory_Vector_3rd(double tf, double tn,
                          Vector3d X, Vector3d dX, Vector3d Xf, Vector3d dXf,
                          Vector3d &Xn, Vector3d &dXn, Vector3d &ddXn);
void PolyTrajectory_Vector_5th(double tf, double tn,
                          Vector3d X, Vector3d dX, Vector3d ddX, Vector3d Xf, Vector3d dXf, Vector3d ddXf,
                          Vector3d &Xn, Vector3d &dXn, Vector3d &ddXn);

void BezierTrajectory_1st(double tf, double tn,
                          double p0, double p1,
                          double &pn, double &dpn, double &ddpn);
void BezierTrajectory_3rd(double tf, double tn,
                          double p0, double p1, double p2, double p3,
                          double &pn, double &dpn, double &ddpn);
void BezierTrajectory_5th(double tf, double tn,
                          double p0, double p1, double p2, double p3, double p4, double p5,
                          double &pn, double &dpn, double &ddpn);

void BezierTrajectory_Vector_1st(double tf, double tn,
                                Vector3d P0, Vector3d P1,
                                Vector3d &Pn, Vector3d &dPn, Vector3d &ddPn);
void BezierTrajectory_Vector_3rd(double tf, double tn,
                                 Vector3d P0, Vector3d P1, Vector3d P2, Vector3d P3,
                                 Vector3d &Pn, Vector3d &dPn, Vector3d &ddPn);
void BezierTrajectory_Vector_5th(double tf, double tn,
                                 Vector3d P0, Vector3d P1, Vector3d P2, Vector3d P3, Vector3d P4, Vector3d P5,
                                 Vector3d &Pn, Vector3d &dPn, Vector3d &ddPn);

enum SolverTypes {
    SolverIsQuadProg = 0,
    SolverIsQPSwift = 1,
};

#define     zero_threshold      1e-12

class QuadEffTest_QP
{

private:
    int Solver;

public:
    int n_X, n_U, n_var;
    int N_window;
    double dT_window;

public:
    MatrixNd A_inequ,B_inequ;
    MatrixNd A_equ,B_equ;
    MatrixNd A_cost,B_cost;
    MatrixNd P_cost,Q_cost;

public:
    QuadEffTest_QP(int SolverSelection = SolverIsQPSwift)
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

    void setNums(int _xlength, int numCost, int numEqConstraints, int numIneqConstraints);
    void make_EQ(MatrixNd A, MatrixNd b);
    void update_Bequ(MatrixNd b);
    void make_IEQ(MatrixNd A, MatrixNd b);
    void update_Binequ(MatrixNd b);
    void make_COST(MatrixNd A, MatrixNd b);
    void update_Bcost(MatrixNd b);
    void make_COST_direct(MatrixNd _P, MatrixNd _Q);
    MatrixNd solve_QP();


    ///// qpSWIFT's Variables ////////////////////////////////////////////////////////////////////////////////

private:
    qp_int  n_var_QPswift;
    qp_int  n_cost_QPswift;
    qp_int  n_equ_QPswift;
    qp_int  n_inequ_QPswift;

    qp_int  exitflag_QPswift;

public:
    qp_int GetNumberOfNonZero_QPswift(MatrixNd mat);
    void ConvertMatrixA_Full2CCS_QPswift(MatrixNd A, qp_real* A_x, qp_int* A_i, qp_int* A_p);
    void ConvertMatrixP_Full2CCS_QPswift(MatrixNd P, qp_real* P_x, qp_int* P_i, qp_int* P_p);
    void ConvertVector2Array_QPswift(VectorNd vec, qp_int n, qp_real* v);
    void ConvertArray2Vector_QPswift(qp_real* v, qp_int n, VectorNd &vec);
    void Example_Problem();
};

void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B);
void Matrix4QP_Addition(MatrixNd& A_base, VectorNd& B_base, MatrixNd A_new, VectorNd B_new);


#endif // QUAD_BASICFUNCTIONS_H
