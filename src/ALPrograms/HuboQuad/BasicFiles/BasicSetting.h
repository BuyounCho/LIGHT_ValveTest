#ifndef BASICSETTING_H
#define BASICSETTING_H


#include "BasicJoint.h"
#include "UserSharedMemory.h"
//#include "RBSharedMemory.h"
//#include "RBLog.h"
//#include "JointInformation.h"


#include <iostream>
#include <libpcan.h>
#include <iostream>
#include <sys/mman.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <alchemy/task.h>

// Basic --------
extern pRBCORE_SHM_COMMAND      sharedCMD;
extern pRBCORE_SHM_REFERENCE    sharedREF;
extern pRBCORE_SHM_SENSOR       sharedSEN;
extern pUSER_SHM                userData;
extern JointControlClass        *jCon;

extern int     __IS_WORKING;
extern int     __IS_GAZEBO;
extern int     PODO_NO;

extern char __AL_NAME[30];

// RT task handler for control
extern RT_TASK rtTaskCon;
extern RT_TASK rtQPTaskCon;
extern RT_TASK rtFlagCon;

// Real-time thread for control
void RBTaskThread(void *);
void RBQPTaskThread(void *);
void RBFlagThread(void *);


using namespace std;
extern void CatchSignals(int _signal);
extern void CheckArguments(int argc, char *argv[]);
extern int RBInitialize(void);

extern int HasAnyOwnership();

#endif // BASICSETTING_H
