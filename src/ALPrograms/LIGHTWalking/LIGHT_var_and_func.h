#ifndef LIGHT3DWALKING_VARIABLES_H
#define LIGHT3DWALKING_VARIABLES_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>

#include "../../../share/Headers/JointInformation.h"
#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/UserSharedMemory.h"
#include "../../../share/Headers/RBLog.h"

#include "rbdl/rbdl.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std::chrono;

#include "LIGHT_qp.h"

// User Function Definition
extern double LPF_1st(double x_old, double x_new, double f_cut);
extern double LPF_2nd(double x_oold, double x_old, double x_new, double f_cut);
extern VectorNd Vector_LPF(VectorNd v, VectorNd v_new, double f_cut);

void fifth_trajectory_oneaxis(double t, double p, double v, double a, double pf, double vf, double af, double &pn, double &vn, double &an);
void third_trajectory_oneaxis(double t, double p, double v, double pf, double vf, double &pn, double &vn);

Matrix3d ExtractRotZ(Matrix3d _R);
Quaternion Quaterion_multiplication(Quaternion A, Quaternion B);
void slerp_trajectory(double s, Quaternion q, Quaternion qf, Quaternion &qn);
void AngularInfo2Quaternion(Matrix3d R, Vector3d w, Vector3d dw, Quaternion &q, Quaternion &dq, Quaternion &ddq);
void Quaternion2AngularInfo(Quaternion q,Matrix3d &R);
Matrix3d VectorCross2Matrix(Vector3d _V);
Matrix3d expMatrix(Matrix3d _M);

int orientation(Vector3d p, Vector3d q, Vector3d r);
void convexHull(Vector3d points[], int n);
//void convexHull(Vector3d points[], int n, MatrixNd &_A, VectorNd &_B);
void Get_SupportConvexHull(int StanceLeg, Vector3d StancePosition, double StanceZAngle,
                           bool StepOn, Vector3d StepPosition, double StepZAngle,
                           MatrixNd& _A, VectorNd& _B);
bool Get_SupportConvexHull_Simplified4Lateral(int StanceLeg, Vector3d StancePosition, bool StepOn,
                                              MatrixNd& _A, VectorNd& _B);

MatrixNd pseudoInverse(MatrixNd & origin);

/********************************************
 * Constant Values
 ********************************************/
#define    MM2M         0.001

/********************************************
 * LIPM dynamics parameter
 ********************************************/
extern double      Pelvis_BaseHeight;
extern double      HeightDiff_CoM2Pel;
extern double      wn_LIPM;

/********************************************
 * Pump Operation Pressure
 ********************************************/
extern const double     PumpPressure_FLOAT;
extern const double     PumpPressure_DSP;
extern const double     PumpPressure_SSP;

/********************************************
 * ETC
 ********************************************/
extern Vector3d      zv;
extern Matrix3d      I3;

#else
#endif // LIGHT3DWALKING_VARIABLES_H


