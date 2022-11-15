#include <math.h>
#include "../../../share/Headers/JointInformation.h"
#include "LIGHT_var_and_func.h"
#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


/********************************************
 * LIPM dynamics parameter
 ********************************************/
double      Pelvis_BaseHeight = 0.70;
double      HeightDiff_CoM2Pel = -0.15;
double      wn_LIPM = sqrt(g_const/(Pelvis_BaseHeight-HeightDiff_CoM2Pel));

/********************************************
 * Pump Operation Pressure
 ********************************************/
const double PumpPressure_FLOAT = 40.0;
const double PumpPressure_DSP = 70.0;
const double PumpPressure_SSP = 100.0;

/********************************************
 * ETC
 ********************************************/
Vector3d zv = Vector3d::Zero();
Matrix3d I3 = Matrix3d::Identity();

