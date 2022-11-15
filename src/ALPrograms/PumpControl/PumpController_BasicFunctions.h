#ifndef PUMPCONTROLLER_BASICFUNCTION_H
#define PUMPCONTROLLER_BASICFUNCTION_H

#include <iostream>
#include <cmath>
#include <chrono>

#include "RBLog.h"
#include "RBSharedMemory.h"
#include "JointInformation.h"
#include "rbdl/rbdl.h"

using namespace std;
using namespace std::chrono;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define     MPatoBAR        10.0   // MPa >> bar
#define     MLPStoLPM       0.06   // mm^2*m/s >> L/min
#define     BARtoPA         100000.0

#define     PUMPSPEED_MAX   2200.0 // @ 100V voltage input
#define     PUMPSPEED_MIN   150.0

#define     K_leak          (0.32/100.0)    // leakage constant [L/min/bar]
#define     n_gas           1.3             // polytropic coefficient
#define     V_pre           0.32            // Pre-charged gas volume [L]
#define     P_pre           70.0            // Pre-charged gas pressure [bar]
#define     P_ModeChange    P_pre
#define     Ps_min          (P_pre+5.0)     // Allowable maximum pressure at accumulator
#define     Ps_max          150.0           // Allowable maximum pressure at accumulator
#define     Ps_margin       3.0

#define     OutputFlowPerRev    (14.0/3000.0)   // 14.0/3000 [L/rev]

// Save Function ------
#define SAVEKIND    30
#define SAVENUM     100000

extern bool             save_PutDataFlag;
extern bool             save_ActivateFlag;
extern unsigned int     save_Index;
extern float            save_Buf[SAVEKIND][SAVENUM];

void PrintHere(int n);
void linear_trajectory_pressure(double tf, double tn, double Pnow, double Pfin, double &Pnext, double &dPnext);


#endif // PUMPCONTROLLER_BASICFUNCTION_H
