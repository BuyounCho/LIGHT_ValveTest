#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicJoint.h"
#include <unistd.h>

// for Rigid Body Dynamics Library
#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define PODO_AL_NAME       "PumpControl"

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *joint;
JointControlClass       *jCon;

extern void PrintHere(int idx);
void save_PutData(unsigned int cur_Index);

// Program variable
int isTerminated;
int     __IS_WORKING = false;
int     __IS_GAZEBO = false;

int     PODO_NO = -1;

int PWM = 0;
char _BNO = 0;

typedef enum PUMP_CONTROL_COMMAND_SET{
    PUMP_CONTROL_NO_ACT = 0,
    PUMP_CONTROL_SAVE_DATA,

}PUMP_CONTROL_COMMAND;

enum _OPERATION_MODE_TYPE
{
    _OPERATION_NO = 200,
};
int _OPERATION_MODE = _OPERATION_NO;
void Ankle_FindHome();
void valve_identification();

// Save Function ------
#define SAVEKIND    6
#define SAVENUM     100000

bool            save_Flag = false;
unsigned int    save_Index = 0;
float           save_Buf[SAVEKIND][SAVENUM];
void            save_PutData(unsigned int cur_Index);
void            save_File();
void            save_File_forID(int _PWM, int direction);


int main(int argc, char *argv[])
{
    // =================== Program initiated ===================
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "SH_TASK");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    jCon->SetMotionOwner(0);

    jCon->RefreshToCurrentReference();
    jCon->SetAllMotionOwner();


    FILE_LOG(logSUCCESS) << "Setting Complete!!";

    // =========================================================

    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){  

        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}


//==============================//
// Task Thread
//==============================//
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {

        switch(_OPERATION_MODE)
        {
        case _OPERATION_NO:
            break;
        default:
            break;
        }

        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);

        if(save_Flag == true){
            save_PutData(save_Index);
            save_Index++;
            if(save_Index == (SAVENUM-1))
                save_Index=0;
        }

    }

}
//==============================//


//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 40*1000);

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
            jCon->JointUpdate();
            rt_task_resume(&rtTaskCon);
        }
    }

}

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = RT_TIMER_PERIOD_MS/1000.0;

    // ankle linear position(unit:mm) data
//    save_Buf[1][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefLinPos;
//    save_Buf[2][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentLinPos;
//    save_Buf[3][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefLinPos;
//    save_Buf[4][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentLinPos;

//    save_Buf[1][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefPWM;
//    save_Buf[2][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentLinPos;
//    save_Buf[3][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentValvePos;

    save_Buf[1][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentRefPWM;
    save_Buf[2][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentAngle;
    save_Buf[3][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentValvePos;
}

void save_File()
{
    FILE* fp;
    unsigned int i, j;

    fp = fopen("PumpControl_Data.txt", "w");

    for(i=0 ; i<save_Index ; i++)
    {
        for(j=0 ; j<SAVEKIND ; j++){
            fprintf(fp, "%f\t", save_Buf[j][i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);

    save_Index=0;
    save_Flag=0;
}


//////=====================================================================================

void valve_identification()
{

}
