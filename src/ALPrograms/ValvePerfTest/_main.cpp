#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicJoint.h"
#include <unistd.h>

// for Quadratic Programming
#include "QP_BasicFiles/QuadProg++.hh"

// for Rigid Body Dynamics Library
#include "rbdl/rbdl.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#include <chrono>
using namespace std;
using namespace std::chrono;

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *joint;
JointControlClass       *jCon;


// Program variable
int isTerminated;
int     __IS_WORKING = false;
int     __IS_GAZEBO = false;

int     PODO_NO = -1;

#define PODO_AL_NAME       "ValvePerfTest"

#define SAVEKIND            30
#define SAVENUM             50000

#define g_const             9.81

bool            save_PutDataFlag = false;
bool            save_ActivateFlag  = false;
unsigned int    save_Index = 0;
unsigned int    save_BNO = 2;
float           save_Buf[SAVEKIND][SAVENUM];
VectorNd        save_Vector = VectorNd::Zero(20);

void    PrintHere(int idx);
void    save_PutData(unsigned int cur_Index);
void    save_File(char* filecomment = "");

void    PosCtrlmodeON(int BNO);
void    TorCtrlmodeON(int BNO, double init_FORCE);
void    ValveOpenmodeON(int BNO, int init_OPEN);
void    PWMmodeON(int BNO, int init_PWM);


void fifth_trajectory_oneaxis(double tf, double tn,
                              double p, double v, double a,
                              double pf, double vf, double af,
                              double &pn, double &vn, double &an);

typedef enum _OPERATION_COMMAND_SET{
    _OPERATION_NO_ACT = 0,
    _OPERATION_VALVEVOLTAGE_SINEWAVE,
    _OPERATION_VALVEPOSITION_SINEWAVE,
    _OPERATION_ACTPOSITION_MOVESMOOTH,
    _OPERATION_ACTPOSITION_SINEWAVE,
    _OPERATION_ACTPOSITION_SINEWAVE_MULTI,
    _OPERATION_ACTFORCE_GRAVCOMP,
    _OPERATION_ACTFORCE_SINEWAVE,

}_OPERATION_COMMAND;

int     _BNO = 0;
int     _OPERATION_MODE = 0;
int     _OPERATION_STAGE = 0;
bool    _OPERATION_STOPSIG = false;
bool    _OPERATION_FINISHED = false;
int     _OPERATION_PARAM_INT[20] = {0,};
double  _OPERATION_PARAM_DOUBLE[20] = {0.0,};

bool    OperFunc_ValveVoltage_SineWave();
bool    OperFunc_ValvePosition_SineWave();
bool    OperFunc_ActPosition_MoveSmooth();
bool    OperFunc_ActPosition_SineWave();
bool    OperFunc_ActPosition_SineWaveMulti();
bool    OperFunc_ActForce_GravComp(bool _STOP_SIG);
bool    OperFunc_ActForce_SineWaveMotion();

double  DebugVar_double[10];

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
    sprintf(__AL_NAME, "ValvePerfTest");

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

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND) {
        case VALVEPERFTEST_SAVE_DATA:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]) {
                save_BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
                save_PutDataFlag = true;
                FILE_LOG(logWARNING) << "SAVING AL DATA START! (BNO : " << save_BNO << ")";
            } else {
                if(save_PutDataFlag) {
                    if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == NULL) {
                        save_File();
                    } else {
                        save_File(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                    }
                    save_PutDataFlag = false;
                    save_ActivateFlag = false;
                    FILE_LOG(logERROR) << "SAVING AL DATA END!!";
                } else {
                    FILE_LOG(logERROR) << "Save Flag is not Activated..";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            break;
        }          

        ///////////////////////////////////////////////////////////////////////////////
        /////  Valve Input Voltage (PWM-duty) Control /////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case VALVEPERFTEST_VALVEVOLTAGE_STEP:
        {
            _BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Selected Board
            int _PWM = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Valve Input Voltage (-12000mV ~ 12000mV)
            PWMmodeON(_BNO,_PWM);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            FILE_LOG(logSUCCESS) << "===============================";
            FILE_LOG(logSUCCESS) << "     < Valve Input Voltage (Step) >";
            FILE_LOG(logSUCCESS) << " BNO : " << _BNO;
            FILE_LOG(logSUCCESS) << " Input Voltage : " << _PWM << " [mV]";
            FILE_LOG(logSUCCESS) << "===============================";
            break;
        }
        case VALVEPERFTEST_VALVEVOLTAGE_SINEWAVE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_VALVEVOLTAGE_SINEWAVE;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /////  Valve Position Control /////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case VALVEPERFTEST_VALVEPOSITION_STEP:
        {
            _BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Selected Board
            int _VALVEPOS = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Valve Position (-10000pulse ~ 10000pulse)
            ValveOpenmodeON(_BNO,_VALVEPOS);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            FILE_LOG(logSUCCESS) << "===============================";
            FILE_LOG(logSUCCESS) << "     < Valve Position Control (Step) >";
            FILE_LOG(logSUCCESS) << " BNO : " << _BNO;
            FILE_LOG(logSUCCESS) << " Valve Position : " << _VALVEPOS << " [pulse]";
            FILE_LOG(logSUCCESS) << "===============================";
            break;
        }
        case VALVEPERFTEST_VALVEPOSITION_SINEWAVE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_VALVEPOSITION_SINEWAVE;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /////  Actuator Position Control //////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case VALVEPERFTEST_ACTPOSITION_MOVESMOOTH:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_ACTPOSITION_MOVESMOOTH;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            break;
        }
        case VALVEPERFTEST_ACTPOSITION_SINEWAVE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_ACTPOSITION_SINEWAVE;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            break;
        }
        case VALVEPERFTEST_ACTPOSITION_SINEWAVE_MULTI:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_ACTPOSITION_SINEWAVE_MULTI;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /////  Actuator Force (Torque) Control ////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case VALVEPERFTEST_ACTFORCE_STEP:
        {
            _BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Selected Board
            double _FORCE = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // Actuator Force
            double _STIFFNESS = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; // Actuator Force
            double _DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]; // Actuator Force
            TorCtrlmodeON(_BNO, _FORCE);
            sharedREF->ActuatorStiffness[_BNO] = _STIFFNESS;
            sharedREF->ActuatorDamping[_BNO] = _DAMPING;
            sharedREF->StiffnDampChange[_BNO] = true;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            FILE_LOG(logSUCCESS) << "===============================";
            FILE_LOG(logSUCCESS) << "   < Actuator Force Control (Step) >";
            FILE_LOG(logSUCCESS) << " BNO : " << _BNO;
            FILE_LOG(logSUCCESS) << " Force  : " << _FORCE << " [N]";
            FILE_LOG(logSUCCESS) << " Actuator Stiffness : " << _STIFFNESS << " [N/mm]";
            FILE_LOG(logSUCCESS) << " Actuator Damping : " << _DAMPING << " [N/(mm/s)]";
            FILE_LOG(logSUCCESS) << "===============================";
            break;
        }

        case VALVEPERFTEST_ACTFORCE_GRAVCOMP:
        {
            _OPERATION_STOPSIG = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            if(_OPERATION_STOPSIG) {
\
            } else {
                _OPERATION_STAGE = 0;
                _OPERATION_MODE = _OPERATION_ACTFORCE_GRAVCOMP;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;

            break;
        }

        case VALVEPERFTEST_ACTFORCE_SINEWAVE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_ACTFORCE_SINEWAVE;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = VALVEPERFTEST_NO_ACT;
            break;
        }

        default:
            break;
        }
    }
    sharedCMD->ALTHREAD_ONOFF_SIGNAL[PODO_NO] = false;
    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}


//==============================//
// Task Thread
//==============================//
int CNT_display = 0;
void *RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        // -----------------------------------
        // Operation
        // -----------------------------------
        bool _FINISH = false;
        switch(_OPERATION_MODE)
        {
        case _OPERATION_NO_ACT:
            break;
        case _OPERATION_VALVEVOLTAGE_SINEWAVE:
            _FINISH = OperFunc_ValveVoltage_SineWave();
            break;
        case _OPERATION_VALVEPOSITION_SINEWAVE:
            _FINISH = OperFunc_ValvePosition_SineWave();
            break;
        case _OPERATION_ACTPOSITION_MOVESMOOTH:
            _FINISH = OperFunc_ActPosition_MoveSmooth();
            break;
        case _OPERATION_ACTPOSITION_SINEWAVE:
            _FINISH = OperFunc_ActPosition_SineWave();
            break;
        case _OPERATION_ACTPOSITION_SINEWAVE_MULTI:
            _FINISH = OperFunc_ActPosition_SineWaveMulti();
            break;
        case _OPERATION_ACTFORCE_GRAVCOMP:
            _FINISH = OperFunc_ActForce_GravComp(_OPERATION_STOPSIG);
            break;
        case _OPERATION_ACTFORCE_SINEWAVE:
            _FINISH = OperFunc_ActForce_SineWaveMotion();
            break;
        default:
            break;
        }

        if (_FINISH == true){
            _OPERATION_MODE = _OPERATION_NO_ACT;
            _OPERATION_FINISHED = false;
            FILE_LOG(logERROR) << " Operation is finished!! ";
        }

        //////======================================================================================

        // -----------------------------------
        // Data Display
        // -----------------------------------
        if (CNT_display >= 200) {
            CNT_display = 0;

        } CNT_display++;

        //////======================================================================================

        if(save_PutDataFlag == true){
            if(save_ActivateFlag == true || save_Index == (SAVENUM-1)) {
                if(save_Index == (SAVENUM-1)) {
                    FILE_LOG(logERROR) << "Save Buffer is Full!!";
                    sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = NULL;
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_SAVE_DATA;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = false;
            } else {
                save_PutData(save_Index);
                save_Index++;
            }
        } else {
            if(save_ActivateFlag == true) {
                save_ActivateFlag = false;
                FILE_LOG(logERROR) << "There is no saved data...";
            }
        }

        //////======================================================================================

        sharedCMD->ALTHREAD_ONOFF_SIGNAL[PODO_NO] = true;
        while(1) {
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) {
                jCon->JointUpdate();
                break;
            } else {
                usleep(2);
            }
        }
    }
    sharedCMD->ALTHREAD_ONOFF_SIGNAL[PODO_NO] = false;
}

//==============================//
// Flag Thread
//==============================//
int CntFlagThread = 0;
void *RBFlagThread(void *)
{

}

//==============================//
// Valve Control Mode Change
//==============================//

void PosCtrlmodeON(int BNO)
{
    double CurrentActPos = sharedSEN->ENCODER[BNO][0].CurrentActPos;
    jCon->SetJointRefActPos(BNO,CurrentActPos);
    jCon->SetJointRefActVel(BNO, 0.0);
    jCon->SetJointRefActForce(BNO, 0.0);

    sharedREF->ValveCtrlMode_Command[BNO] = ValveControlMode_PosOrFor;
    sharedREF->PosOrFor_Selection[BNO] = JointControlMode_Position;
}

void TorCtrlmodeON(int BNO, double init_FORCE)
{
    double CurrentActPos = sharedSEN->ENCODER[BNO][0].CurrentActPos;
    jCon->SetJointRefActPos(BNO,CurrentActPos);
    jCon->SetJointRefActVel(BNO, 0.0);
    jCon->SetJointRefActForce(BNO, init_FORCE);

    sharedREF->ValveCtrlMode_Command[BNO] = ValveControlMode_PosOrFor;
    sharedREF->PosOrFor_Selection[BNO] = JointControlMode_Torque;
}

void ValveOpenmodeON(int BNO, int init_OPEN)
{
    jCon->SetRefValvePos(BNO, init_OPEN);
    sharedREF->ValveCtrlMode_Command[BNO] = ValveControlMode_Opening;
}

void PWMmodeON(int BNO, int init_PWM)
{
    jCon->SetRefPWM(BNO, init_PWM);
    sharedREF->ValveCtrlMode_Command[BNO] = ValveControlMode_PWM;
}


//==============================//
// Operational Functions
//==============================//

bool    OperFunc_ValveVoltage_SineWave()
{
    static int BNO;
    static double SINE_MAG;
    static double SINE_NUM;
    static double SINE_PER;
    static double t = 0.0;

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Board Number
        save_BNO = BNO;
        SINE_MAG = (double)sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // mV
        SINE_NUM = (double)sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]; // Number
        SINE_PER = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // Period

        PWMmodeON(BNO,0);

        FILE_LOG(logSUCCESS) << "===============================";
        FILE_LOG(logSUCCESS) << "     < Valve Input Voltage (Sine Wave) >";
        FILE_LOG(logSUCCESS) << " BNO : " << _BNO;
        FILE_LOG(logSUCCESS) << " Voltage Magnitude : " << SINE_MAG << " [mV]";
        FILE_LOG(logSUCCESS) << " Number : " << SINE_NUM << " [times]";
        FILE_LOG(logSUCCESS) << " Period : " << SINE_PER << " [sec]";
        FILE_LOG(logSUCCESS) << "===============================";

        t = 0.0;
        save_PutDataFlag = true;
        _OPERATION_STAGE++;

    } else if(_OPERATION_STAGE == 1) {

        double Ref_PWM = (double)SINE_MAG*sin(2.0*PI*t/SINE_PER);
        int _PWM = (int)Ref_PWM;
        jCon->SetRefPWM(BNO,_PWM);

        if (t >= SINE_NUM*SINE_PER) {
            t = 0.0;
            save_ActivateFlag = true;
            char filename[30] = "SineWaveVoltage_";
            strcat(filename, JointNameList[BNO].toLocal8Bit().data());
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_VALVETEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }
}

bool    OperFunc_ValvePosition_SineWave()
{
    static int BNO;
    static double SINE_MAG;
    static double SINE_NUM;
    static double SINE_PER;
    static double t = 0.0;

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Board Number
        save_BNO = BNO;
        SINE_MAG = (double)sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // pulse
        SINE_NUM = (double)sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]; // Number
        SINE_PER = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // Period

        ValveOpenmodeON(BNO,0);

        FILE_LOG(logSUCCESS) << "===============================";
        FILE_LOG(logSUCCESS) << "     < Valve Position Control (Sine Wave) >";
        FILE_LOG(logSUCCESS) << " BNO : " << _BNO;
        FILE_LOG(logSUCCESS) << " ValvePos Magnitude : " << SINE_MAG << " [pulse]";
        FILE_LOG(logSUCCESS) << " Number : " << SINE_NUM << " [times]";
        FILE_LOG(logSUCCESS) << " Period : " << SINE_PER << " [sec]";
        FILE_LOG(logSUCCESS) << "===============================";

        t = 0.0;
        save_PutDataFlag = true;
        _OPERATION_STAGE++;

    } else if(_OPERATION_STAGE == 1) {

        double Ref_ValvePos = (double)SINE_MAG*sin(2.0*PI*t/SINE_PER);
        int _ValvePos = (int)Ref_ValvePos;
        jCon->SetRefValvePos(BNO,_ValvePos);

        if (t >= SINE_NUM*SINE_PER) {
            t = 0.0;
            save_ActivateFlag = true;
            char filename[30] = "SineWaveValvePos_";
            strcat(filename, JointNameList[BNO].toLocal8Bit().data());
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_VALVETEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }
}

bool    OperFunc_ActPosition_MoveSmooth()
{
    static int BNO;
    static double DES_POS;
    static double INIT_POS;
    static double MOVE_TIME;
    static double t = 0.0;

    if(_OPERATION_STAGE == 0) {
        BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Joint Number
        INIT_POS = sharedSEN->ENCODER[BNO][0].CurrentActPos;
        DES_POS = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // mm or deg
        MOVE_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; //sec
        PosCtrlmodeON(BNO);

        FILE_LOG(logSUCCESS) << "===============================";
        FILE_LOG(logSUCCESS) << "     < Actuator Position Control (Move Smoothly) >";
        FILE_LOG(logSUCCESS) << " BNO : " << BNO;
        FILE_LOG(logSUCCESS) << " Desired Position : " << DES_POS << " [mm (or deg)]";
        FILE_LOG(logSUCCESS) << " Moving Time : " << MOVE_TIME << " [sec]";
        FILE_LOG(logSUCCESS) << "===============================";

        t = 0.0;
        _OPERATION_STAGE++;
    } else if(_OPERATION_STAGE == 1) {
        double Ref_Pos = (DES_POS-INIT_POS)*(1.0-cos(PI*t/MOVE_TIME))/2.0 + INIT_POS;
        double Ref_Vel = (DES_POS-INIT_POS)*(PI/MOVE_TIME)*sin(PI*t/MOVE_TIME)/2.0;
        jCon->SetJointRefActPos(BNO,Ref_Pos);
        jCon->SetJointRefActVel(BNO,Ref_Vel);

        if (t >= MOVE_TIME) {
            jCon->SetJointRefActVel(BNO,0.0);
            t = 0.0;
            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_VALVETEST;

    if(_OPERATION_FINISHED) {
        return true;
    } else { return false; }
}

bool    OperFunc_ActPosition_SineWave()
{
    static int BNO;
    static double INIT_POS;
    static double SINE_MAG;
    static double SINE_NUM;
    static double SINE_PER;
    static double t = 0.0;

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Board Number
        save_BNO = BNO;
        INIT_POS = sharedSEN->ENCODER[BNO][0].CurrentActPos;
        SINE_MAG = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // mm (or deg)
        SINE_NUM = (double)sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Number
        SINE_PER = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; // Period
        PosCtrlmodeON(BNO);

        FILE_LOG(logSUCCESS) << "===============================";
        FILE_LOG(logSUCCESS) << "     < Valve Position Control (Sine Wave) >";
        FILE_LOG(logSUCCESS) << " BNO : " << BNO;
        FILE_LOG(logSUCCESS) << " Act. Pos, Magnitude : " << SINE_MAG << " [mm (or deg)]";
        FILE_LOG(logSUCCESS) << " Number : " << SINE_NUM << " [times]";
        FILE_LOG(logSUCCESS) << " Period : " << SINE_PER << " [sec]";
        FILE_LOG(logSUCCESS) << "===============================";

        t = 0.0;
        save_PutDataFlag = true;
        _OPERATION_STAGE++;

    } else if(_OPERATION_STAGE == 1) {

        double Ref_Pos = SINE_MAG*sin(2.0*PI*t/SINE_PER) + INIT_POS;
        double Ref_Vel = SINE_MAG*(2.0*PI/SINE_PER)*cos(2.0*PI*t/SINE_PER);
        jCon->SetJointRefActPos(BNO,Ref_Pos);
        jCon->SetJointRefActVel(BNO,Ref_Vel);

        if (t >= SINE_NUM*SINE_PER) {
            jCon->SetJointRefActVel(BNO,0.0);
            t = 0.0;
            save_ActivateFlag = true;
            char filename[30] = "SineWaveActPos_";
            strcat(filename, JointNameList[BNO].toLocal8Bit().data());
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_VALVETEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }
}

bool    OperFunc_ActPosition_SineWaveMulti()
{
    static int BNO;
    static double INIT_POS;
    static double SINE_MAG;
    static double SINE_NUM;
    static double SINE_PER;
    static double t = 0.0;

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Board Number
        save_BNO = BNO;
        INIT_POS = sharedSEN->ENCODER[BNO][0].CurrentActPos;
        SINE_MAG = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // mm (or deg)
        SINE_NUM = (double)sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Number
        SINE_PER = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; // Period
        PosCtrlmodeON(BNO);

        FILE_LOG(logSUCCESS) << "===============================";
        FILE_LOG(logSUCCESS) << "     < Valve Position Control (Sine Wave) >";
        FILE_LOG(logSUCCESS) << " BNO : " << _BNO;
        FILE_LOG(logSUCCESS) << " Act. Pos, Magnitude : " << SINE_MAG << " [mm (or deg)]";
        FILE_LOG(logSUCCESS) << " Number : " << SINE_NUM << " [times]";
        FILE_LOG(logSUCCESS) << " Period : " << SINE_PER << " [sec]";
        FILE_LOG(logSUCCESS) << "===============================";

        t = 0.0;
        save_PutDataFlag = true;
        _OPERATION_STAGE++;

    } else if(_OPERATION_STAGE == 1) {

        double Ref_Pos = SINE_MAG*sin(2.0*PI*t/SINE_PER) + INIT_POS;
        double Ref_Vel = SINE_MAG*(2.0*PI/SINE_PER)*cos(2.0*PI*t/SINE_PER);
        jCon->SetJointRefActPos(BNO,Ref_Pos);
        jCon->SetJointRefActVel(BNO,Ref_Vel);

        if (t >= SINE_NUM*SINE_PER) {
            jCon->SetJointRefActVel(BNO,0.0);
            t = 0.0;
            save_ActivateFlag = true;
            char filename[30] = "SineWaveActPos_";
            strcat(filename, JointNameList[BNO].toLocal8Bit().data());
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_VALVETEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }

}

bool    OperFunc_ActForce_GravComp(bool _STOP_SIG)
{
    static int BNO;
    static double LOAD_MASS;

    double L = 0.40; // Link Mass
    double th_off = 10.0; // homepose offset

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Board Number
        LOAD_MASS = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // mass, kg

        double _STIFFNESS = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; // Actuator Force
        double _DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]; // Actuator Force
        sharedREF->ActuatorStiffness[BNO] = _STIFFNESS;
        sharedREF->ActuatorDamping[BNO] = _DAMPING;
        sharedREF->StiffnDampChange[BNO] = true;

        double Ref_Pos = sharedSEN->ENCODER[BNO][0].CurrentActPos;
        double Fref = -LOAD_MASS*g_const*L*sin((Ref_Pos+th_off)*D2R);
        TorCtrlmodeON(BNO, Fref);

        FILE_LOG(logSUCCESS) << "===============================";
        FILE_LOG(logSUCCESS) << "     < Gravity compensation with force ctrl. >";
        FILE_LOG(logSUCCESS) << " BNO : " << BNO;
        FILE_LOG(logSUCCESS) << " Load Mass : " << LOAD_MASS << " [kg]";
        FILE_LOG(logSUCCESS) << " Actuator Stiffness : " << _STIFFNESS << " [N/mm]";
        FILE_LOG(logSUCCESS) << " Actuator Damping : " << _DAMPING << " [N/(mm/s)]";
        FILE_LOG(logSUCCESS) << "===============================";

        _OPERATION_STAGE++;

    } else if(_OPERATION_STAGE == 1) { // Grav. Compensation start!

        double Ref_Pos = sharedSEN->ENCODER[BNO][0].CurrentActPos;
        double Fref = -LOAD_MASS*g_const*L*sin((Ref_Pos+th_off)*D2R);
        jCon->SetJointRefActForce(BNO, Fref);

        DebugVar_double[0] = Fref;

        FILE_LOG(logINFO) << "Fref : " << Fref;

        if (_STOP_SIG) {
            PosCtrlmodeON(BNO);
            FILE_LOG(logERROR) << "Gravity Compensation is stopped!!";

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    if(_OPERATION_FINISHED) {
        return true;
    } else { return false; }
}

bool    OperFunc_ActForce_SineWaveMotion()
{
    static int BNO;
    static double INIT_POS;
    static double SINE_MASS;
    static double SINE_MAG;
    static double SINE_NUM;
    static double SINE_PER;
    static double t = 0.0;

    double L = 0.40; // Link Mass
    double th_off = 10.0; // homepose offset

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]; // Board Number
        save_BNO = BNO;
        INIT_POS = sharedSEN->ENCODER[BNO][0].CurrentActPos;
        SINE_MASS = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // mass, kg

        double _STIFFNESS = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; // Actuator Force
        double _DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]; // Actuator Force
        sharedREF->ActuatorStiffness[BNO] = _STIFFNESS;
        sharedREF->ActuatorDamping[BNO] = _DAMPING;
        sharedREF->StiffnDampChange[BNO] = true;

        SINE_NUM = (double)sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Number
        SINE_MAG = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3]; // mm (or deg)
        SINE_PER = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4]; // Period

        FILE_LOG(logSUCCESS) << "===============================";
        FILE_LOG(logSUCCESS) << "     < Sine Wave Motion with Force Ctrl. >";
        FILE_LOG(logSUCCESS) << " BNO : " << BNO;
        FILE_LOG(logSUCCESS) << " Load Mass : " << SINE_MASS << " [kg]";
        FILE_LOG(logSUCCESS) << " Actuator Stiffness : " << _STIFFNESS << " [N/mm]";
        FILE_LOG(logSUCCESS) << " Actuator Damping : " << _DAMPING << " [N/(mm/s)]";
        FILE_LOG(logSUCCESS) << " Act. Pos, Magnitude : " << SINE_MAG << " [mm (or deg)]";
        FILE_LOG(logSUCCESS) << " Number : " << SINE_NUM << " [times]";
        FILE_LOG(logSUCCESS) << " Period : " << SINE_PER << " [sec]";
        FILE_LOG(logSUCCESS) << "===============================";

        t = 0.0;
        save_PutDataFlag = true;
        _OPERATION_STAGE++;

    } else if(_OPERATION_STAGE == 1) { // Smooth motion start

        double Ref_Pos = SINE_MAG + INIT_POS;
        double Ref_Vel = 0.0;
        double Ref_Acc = -SINE_MAG*(2.0*PI/SINE_PER)*(2.0*PI/SINE_PER);
        double nRef_Pos, nRef_Vel, nRef_Acc;

        double T = SINE_PER/3.0;
        fifth_trajectory_oneaxis(T,t,
                                 INIT_POS,0.0,0.0,
                                 Ref_Pos,Ref_Vel,Ref_Acc,
                                 nRef_Pos, nRef_Vel, nRef_Acc);
        jCon->SetJointRefActPos(BNO,nRef_Pos);
        jCon->SetJointRefActVel(BNO,nRef_Vel);

        double Fref = SINE_MASS*L*L*nRef_Acc*D2R - SINE_MASS*g_const*L*sin((nRef_Pos+th_off)*D2R);
        jCon->SetJointRefActForce(BNO, Fref);

        DebugVar_double[0] = Fref;
        DebugVar_double[1] = nRef_Pos;
        DebugVar_double[2] = nRef_Vel;

        if (t >= T) {
            t = 0.0;
            _OPERATION_STAGE++;
        }
    } else if(_OPERATION_STAGE == 2) {  // Mid motion

        double Ref_Pos = SINE_MAG*cos(2.0*PI*t/SINE_PER) + INIT_POS;
        double Ref_Vel = -SINE_MAG*(2.0*PI/SINE_PER)*sin(2.0*PI*t/SINE_PER);
        double Ref_Acc = -SINE_MAG*(2.0*PI/SINE_PER)*(2.0*PI/SINE_PER)*cos(2.0*PI*t/SINE_PER);
        jCon->SetJointRefActPos(BNO,Ref_Pos);
        jCon->SetJointRefActVel(BNO,Ref_Vel);

        double Fref = SINE_MASS*L*L*Ref_Acc*D2R - SINE_MASS*g_const*L*sin((Ref_Pos+th_off)*D2R);
        jCon->SetJointRefActForce(BNO, Fref);

        DebugVar_double[0] = Fref;
        DebugVar_double[1] = Ref_Pos;
        DebugVar_double[2] = Ref_Vel;

        if (t >= SINE_NUM*SINE_PER) {
            t = 0.0;
            _OPERATION_STAGE++;
        }
    } else if(_OPERATION_STAGE == 3) { // Smooth motion finish

        double iRef_Pos = SINE_MAG+INIT_POS;
        double iRef_Vel = 0.0;
        double iRef_Acc = -SINE_MAG*(2.0*PI/SINE_PER)*(2.0*PI/SINE_PER);
        double nRef_Pos, nRef_Vel, nRef_Acc;

        double T = SINE_PER/3.0;
        fifth_trajectory_oneaxis(T,t,
                                 iRef_Pos,iRef_Vel,iRef_Acc,
                                 INIT_POS,0.0,0.0,
                                 nRef_Pos, nRef_Vel, nRef_Acc);
        jCon->SetJointRefActPos(BNO,nRef_Pos);
        jCon->SetJointRefActVel(BNO,nRef_Vel);

        double Fref = SINE_MASS*L*L*nRef_Acc*D2R - SINE_MASS*g_const*L*sin((nRef_Pos+th_off)*D2R);
        jCon->SetJointRefActForce(BNO, Fref);

        DebugVar_double[0] = Fref;
        DebugVar_double[1] = nRef_Pos;
        DebugVar_double[2] = nRef_Vel;

        if (t >= T) {
            jCon->SetJointRefActVel(BNO,0.0);
            t = 0.0;
            save_ActivateFlag = true;
            char filename[30] = "ForCtrl_SineMotion_";
            strcat(filename, JointNameList[BNO].toLocal8Bit().data());
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_VALVETEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }
}



//==============================//
// Trajectory Generation
//==============================//

void fifth_trajectory_oneaxis(double tf, double tn,
                              double p, double v, double a,
                              double pf, double vf, double af,
                              double &pn, double &vn, double &an)
{
    // t : moving time
    // p, v, a : current position, velocity, acceleration
    // pf, vf, af : final position, velocity, acceleration
    // pn, vn, an : next step position, velocity, acceleration

    double k[6] = {0.0, }; // Coefficient for 'fifth order polynomial trajectory'
    double theta = p,theta_dot = v,theta_ddot = a;
    double theta_f = pf,theta_dot_f = vf,theta_ddot_f = af;

    // ref(0) : Position
    // ref(1) : Velocityr
    // ref(2) : Acceleration
    if(tf < 1e-6) {
        pn = theta;
        vn = theta_dot;
        an = theta_ddot;
    } else if (tf < tn) {
        pn = pf;
        vn = vf;
        an = af;
    } else {
        double t = tf;
        k[0] = theta;
        k[1] = theta_dot;
        k[2] = theta_ddot/2.;
        k[3] = (20.*(theta_f-theta) - (8.*theta_dot_f + 12.*theta_dot)*t-(3.*theta_ddot - theta_ddot_f)*t*t)/(2.*t*t*t);
        k[4] = ((30.*theta - 30.*theta_f) + (14.*theta_dot_f+16.*theta_dot)*t+(3.*theta_ddot-2.*theta_ddot_f)*t*t)/(2.*t*t*t*t);
        k[5] = (12.*theta_f - 12.*theta - (6.*theta_dot_f + 6.*theta_dot)*t - (theta_ddot-theta_ddot_f)*t*t)/(2.*t*t*t*t*t);

        double dt = tn;
        pn = k[0] + k[1]*dt  + k[2]*dt*dt + k[3]*dt*dt*dt + k[4]*dt*dt*dt*dt + k[5]*dt*dt*dt*dt*dt;
        vn = k[1] + 2.0*k[2]*dt + 3.0*k[3]*dt*dt + 4.0*k[4]*dt*dt*dt + 5.0*k[5]*dt*dt*dt*dt;
        an = 2.0*k[2] + 6.0*k[3]*dt + 12.0*k[4]*dt*dt + 20.0*k[5]*dt*dt*dt;
    }
}
//==============================//
// Saving & Debuging Functions
//==============================//

void PrintHere(int n)
{
    static int cnt = 0;
    if(n == 0) {
        FILE_LOG(logDEBUG1) << "I'm Here : " << cnt;
        cnt++;
    } else {
        FILE_LOG(logDEBUG) << "I'm Here : " << n;
    }
}

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = SYS_DT_ACTTEST;

    // RIGHT HIP YAW (Board 0)
    save_Buf[1][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentRefActPos;
    save_Buf[2][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentActPos;
    save_Buf[3][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentRefActVel;
    save_Buf[4][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentActVel;
    save_Buf[5][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentRefActForce;
    save_Buf[6][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentActForce;

    save_Buf[7][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentRefValvePos;
    save_Buf[8][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentValvePosRef;
    save_Buf[9][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentValvePos;

    save_Buf[10][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentRefPWM;
    save_Buf[11][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentPWM;

    save_Buf[12][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentData1;
    save_Buf[13][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentData2;
    save_Buf[14][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentData3;
    save_Buf[15][cur_Index] = sharedSEN->ENCODER[save_BNO][0].CurrentData4;

    save_Buf[16][cur_Index] = DebugVar_double[0];
    save_Buf[17][cur_Index] = DebugVar_double[1];
    save_Buf[18][cur_Index] = DebugVar_double[2];

}

void save_File(char *filecomment)
{
    int n_col = save_Index;
    save_Index = 0;

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "DataLog/ValvePerfTest_Data/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();

    const char* filetype = ".txt";
    char filename[100];

    strcpy(filename, filedate);
    if(!filecomment[0] == NULL) {
        strcat(filename, "_");
        strcat(filename, filecomment);
    }
    strcat(filename, filetype);

    FILE* fp;
    unsigned int i, j;
    fp = fopen(filename, "w");

    for(i=0 ; i<n_col ; i++)
    {
        for(j=0 ; j<SAVEKIND ; j++){
            fprintf(fp, "%f\t", save_Buf[j][i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    save_Index = 0;
    std::cout << "Saved Filename : " << filename << std::endl;
}
