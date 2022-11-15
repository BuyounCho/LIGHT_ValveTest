#ifndef EXTERNHEADER_H
#define EXTERNHEADER_H

// Devices --------
#include "HydraulicActuatorController.h"
#include "RBFTSensor.h"
#include "RBIMUSensor.h"
//#include "RBSmartPower.h"
//#include "RBOpticFlowSensor.h"
//#include "RBFOGSensor.h"

extern ValveController      _DEV_VC[MAX_VC];
extern RBFTSensor          _DEV_FT[MAX_FT];
extern RBIMUSensor         _DEV_IMU[MAX_IMU];
//extern RBSmartPower        _DEV_SP[MAX_SP];
//extern RBOpticFlowSensor   _DEV_OF[MAX_OF];
//extern RBFOGSensor         _DEV_FOG;

extern int     _VERSION;
extern int     _NO_OF_AL;
extern int     _NO_OF_COMM_CH;
extern int     _NO_OF_VC;
extern int     _NO_OF_FT;
extern int     _NO_OF_IMU;
//extern int     _NO_OF_SP;
//extern int     _NO_OF_OF;

//extern int  __IS_GAZEBO;
//extern int  __IS_ROS;

#endif // EXTERNHEADER_H
