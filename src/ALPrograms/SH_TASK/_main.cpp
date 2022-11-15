#include "BasicFiles/BasicSetting.h"

#include "LIGHT_jointsetmodel.h"
#include "SH_TASK_BasicFunction.h"

#include <unistd.h>
#include <cstdlib>

// for Rigid Body Dynamics Library
#include "rbdl/rbdl.h"

// for Quadratic Programming
#include "QP_BasicFiles/QuadProg++.hh"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define PODO_AL_NAME       "SH_TASK"

bool            save_PutDataFlag = false;
bool            save_ActivateFlag  = false;
unsigned int    save_Index = 0;
float           save_Buf[SAVEKIND][SAVENUM];
VectorNd        save_Vector = VectorNd::Zero(20);

bool            save_Flag_forID;
unsigned int    save_Index_forID;
float           save_Buf_forID[SAVEKIND_FORID][SAVENUM];
int             ID_BNO;

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *jCon;

// LIGHT Model
INFO_LIGHT              LIGHT_Info;
LIGHTJointSet           LIGHTJoints;

// Program variable
int isTerminated;
int     __IS_WORKING = false;

int     PODO_NO = -1;

int PWM = 0;
int _BNO = 0;

enum _OPERATION_MODE_TYPE
{
    _OPERATION_NO = 200,

    _OPERATION_VALVEID,

    _OPERATION_SINEWAVE_OPENING,
    _OPERATION_JUMP_OPENING,

    _OPERATION_POSITION_CONTROL_HOMEPOSE,
    _OPERATION_POSITION_CONTROL_WALKREADYPOSE,
    _OPERATION_POSITION_CONTROL_GOTO_POSE,
    _OPERATION_POSITION_CONTROL_RANDOM_MOTION,
    _OPERATION_POSITION_CONTROL_SINEWAVE,

    _OPERATION_TORQUE_CONTROL_CONSTANT_TORQUE,


};

int     _OPERATION_MODE = _OPERATION_NO;

int     _OPERATION_ONOFF = false;
bool    _OPERATION_FINISHED = false;

double    _OPERATION_SINE_MAG = 10.0;
double    _OPERATION_SINE_PER = 10.0;
int       _OPERATION_SINE_NUM = 5;

double    _OPERATION_PARA_DOUBLE[10] = {0.0,};
double    _OPERATION_PARA_INT[10] = {0,};

extern unsigned int     _OPERATION_STAGE = 0;


void    PosCtrlmodeON(int BNO);
void    TorCtrlmodeON(int BNO);
void    ValveOpenmodeON(int BNO, int init_OPEN);

void    Generate_PumpPressureRef(VectorNd Qref, VectorNd dQref, VectorNd Tref, VectorNd &Qact, VectorNd &Pload);

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
        case SH_TASK_SAVE_DATA:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]) {
                save_PutDataFlag = true;
                FILE_LOG(logWARNING) << "SAVING AL DATA START!!";
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
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /////  Valve Opening (No Control) /////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case SH_TASK_VALVE_ID_POS:
        {
            string ID_Date;
            auto now = system_clock::now();
            auto in_time_t = system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%X/");
            ID_Date = ss.str();
            ID_Date.erase(ID_Date.end()-7);
            ID_Date.erase(ID_Date.end()-4);

            int OPEN_MIN     = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            int OPEN_RESOL   = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            int OPEN_MAX     = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
            double PRES_MIN     = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double PRES_RESOL   = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            double PRES_MAX     = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            LIGHTJoints.ValveIdentification_ParameterSet(ID_Date, +1,
                                                         OPEN_MIN,OPEN_RESOL,OPEN_MAX,
                                                         PRES_MIN,PRES_RESOL,PRES_MAX);

            FILE_LOG(logSUCCESS) << "===============================";
            FILE_LOG(logSUCCESS) << "     < Valve ID Start >";
            FILE_LOG(logSUCCESS) << " OPEN_MIN : " << OPEN_MIN;
            FILE_LOG(logSUCCESS) << " OPEN_RESOL : " << OPEN_RESOL;
            FILE_LOG(logSUCCESS) << " OPEN_MAX : " << OPEN_MAX;
            FILE_LOG(logSUCCESS) << " PRES_MIN : " << PRES_MIN;
            FILE_LOG(logSUCCESS) << " PRES_RESOL : " << PRES_RESOL;
            FILE_LOG(logSUCCESS) << " PRES_MAX : " << PRES_MAX;
            FILE_LOG(logSUCCESS) << "===============================";

            _OPERATION_MODE = _OPERATION_VALVEID;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_VALVE_ID_NEG:
        {
            string ID_Date;
            auto now = system_clock::now();
            auto in_time_t = system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%X/");
            ID_Date = ss.str();
            ID_Date.erase(ID_Date.end()-7);
            ID_Date.erase(ID_Date.end()-4);

            int OPEN_MIN     = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            int OPEN_RESOL   = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            int OPEN_MAX     = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
            double PRES_MIN     = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double PRES_RESOL   = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            double PRES_MAX     = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            LIGHTJoints.ValveIdentification_ParameterSet(ID_Date, -1,
                                                         OPEN_MIN,OPEN_RESOL,OPEN_MAX,
                                                         PRES_MIN,PRES_RESOL,PRES_MAX);

            FILE_LOG(logSUCCESS) << "===============================";
            FILE_LOG(logSUCCESS) << "     < Valve ID Start >";
            FILE_LOG(logSUCCESS) << " OPEN_MIN : " << OPEN_MIN;
            FILE_LOG(logSUCCESS) << " OPEN_RESOL : " << OPEN_RESOL;
            FILE_LOG(logSUCCESS) << " OPEN_MAX : " << OPEN_MAX;
            FILE_LOG(logSUCCESS) << " PRES_MIN : " << PRES_MIN;
            FILE_LOG(logSUCCESS) << " PRES_RESOL : " << PRES_RESOL;
            FILE_LOG(logSUCCESS) << " PRES_MAX : " << PRES_MAX;
            FILE_LOG(logSUCCESS) << "===============================";

            _OPERATION_MODE = _OPERATION_VALVEID;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }


        ///////////////////////////////////////////////////////////////////////////////
        /////  Valve Opening (Openloop Control) /////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case SH_TASK_CONST_OPENING:
        {
            _BNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Selected Board
            int _OPENING = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2]; // Valve Opening(-10000~10000)
            if(_BNO == 4||_BNO == 5) {
                ValveOpenmodeON(4,_OPENING);
                ValveOpenmodeON(5,_OPENING);
            } else if (_BNO == 10||_BNO == 11) {
                ValveOpenmodeON(10,_OPENING);
                ValveOpenmodeON(11,_OPENING);
            } else {
                ValveOpenmodeON(_BNO,_OPENING);
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_SINEWAVE_OPENING:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_SINEWAVE_OPENING;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_JUMP_OPENING:
        {
            _OPERATION_MODE = _OPERATION_JUMP_OPENING;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /////  Position Control ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case SH_TASK_POSITION_CONTROL_HOMEPOSE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_POSITION_CONTROL_HOMEPOSE;
            FILE_LOG(logWARNING) << "Position Control : Homepose Start!";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_POSITION_CONTROL_WALKREADYPOSE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_POSITION_CONTROL_WALKREADYPOSE;
            FILE_LOG(logWARNING) << "Position Control : Walk Ready Start!";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_POSITION_CONTROL_GOTO_POSE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_POSITION_CONTROL_GOTO_POSE;
            FILE_LOG(logWARNING) << "Position Control : Go to Position Start!";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_POSITION_CONTROL_RANDOM_MOTION:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_POSITION_CONTROL_RANDOM_MOTION;
            FILE_LOG(logWARNING) << "Position Control : Random Motion Start!";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_POSITION_CONTROL_SINEWAVE:
        {
            _OPERATION_STAGE = 0;
            _OPERATION_MODE = _OPERATION_POSITION_CONTROL_SINEWAVE;
            FILE_LOG(logWARNING) << "Position Control : Sine Wave Start!";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /////  Torque Control ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        case SH_TASK_TORQUE_CONTROL_GO:
        {
            _OPERATION_MODE = _OPERATION_TORQUE_CONTROL_CONSTANT_TORQUE;
            _OPERATION_ONOFF = false;

            _OPERATION_PARA_DOUBLE[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            _OPERATION_PARA_DOUBLE[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            _OPERATION_PARA_DOUBLE[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            _OPERATION_PARA_INT[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
            break;
        }

        case SH_TASK_TORQUE_CONTROL_STOP:
        {
            _OPERATION_MODE = _OPERATION_TORQUE_CONTROL_CONSTANT_TORQUE;
            _OPERATION_ONOFF = true;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SH_TASK_NO_ACT;
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
        //        static timespec TIME_TIC;
        //        static timespec TIME_TOC;
        //        static int CNT_TICTOC = 0;
        //        CNT_TICTOC++;
        //        if(CNT_TICTOC%2==0) {
        //            clock_gettime(CLOCK_REALTIME, &TIME_TIC);
        //            cout << "Cycle Time (SH_TASK Thread) : " << timediff_us(&TIME_TOC, &TIME_TIC)*0.001 << " ms " << endl;
        //            CNT_TICTOC = 0;
        //        } else {
        //            clock_gettime(CLOCK_REALTIME, &TIME_TOC);
        //            cout << "Cycle Time (SH_TASK Thread) : " << timediff_us(&TIME_TIC, &TIME_TOC)*0.001 << " ms " << endl;
        //        }

        // -----------------------------------
        // Get joint position(sensor data) and LIGHT Qnow Update (real-time)
        // -----------------------------------
        LIGHTJoints.UpdateJointPosition();

        // -----------------------------------
        // Operation
        // -----------------------------------
        bool _FINISH = false;
        switch(_OPERATION_MODE)
        {
        case _OPERATION_NO:
            break;
        case _OPERATION_VALVEID:
            _FINISH = LIGHTJoints.ValveIdentification_VariablePressure(true);
            break;
        case _OPERATION_SINEWAVE_OPENING:
            _FINISH = LIGHTJoints.ValveOpenCtrl_SineWave();
            break;
        case _OPERATION_JUMP_OPENING:
            _FINISH = LIGHTJoints.ValveOpenCtrl_Jump();
            break;
        case _OPERATION_POSITION_CONTROL_HOMEPOSE:
            _FINISH = LIGHTJoints.PosCtrl_HomePose();
            break;
        case _OPERATION_POSITION_CONTROL_WALKREADYPOSE:
            _FINISH = LIGHTJoints.PosCtrl_WalkReadyPose();
            break;
        case _OPERATION_POSITION_CONTROL_GOTO_POSE:
            _FINISH = LIGHTJoints.PosCtrl_GOTO_Pose();
            break;
        case _OPERATION_POSITION_CONTROL_RANDOM_MOTION:
            _FINISH = LIGHTJoints.PosCtrl_Random_Motion();
            break;
        case _OPERATION_POSITION_CONTROL_SINEWAVE:
            _FINISH = LIGHTJoints.PosCtrl_Sine_Motion();
            break;
        case _OPERATION_TORQUE_CONTROL_CONSTANT_TORQUE:
            _FINISH = LIGHTJoints.TorCtrl_ConstantTorque(_OPERATION_PARA_DOUBLE[0],
                    _OPERATION_PARA_DOUBLE[1], _OPERATION_PARA_DOUBLE[2], _OPERATION_ONOFF);
            break;
        default:
            break;
        }

        if (_FINISH == true){
            _OPERATION_MODE = _OPERATION_NO;
            _OPERATION_FINISHED = false;
            LIGHTJoints.dQref = MatrixNd::Zero(LIGHTJoints.n_dof,1);
            FILE_LOG(logERROR) << " Operation is finished!! ";
        }


//        VectorNd _Q = VectorNd::Zero(LIGHT_ACT_DOF);
//        VectorNd _dQ = VectorNd::Zero(LIGHT_ACT_DOF);
//        VectorNd _T = 100.0*VectorNd::Ones(LIGHT_ACT_DOF);
//        VectorNd Qact = VectorNd::Zero(LIGHT_ACT_DOF);
//        VectorNd Pload = VectorNd::Zero(LIGHT_ACT_DOF);
//        _T(RAR) = 0.0;
//        _T(LAR) = 0.0;
//        Generate_PumpPressureRef(_Q,_dQ,_T,Qact,Pload);

        //        cout << "Pload : " << Pload.transpose() << endl;

        // -----------------------------------
        // LIGHT Qref Update (real-time)
        // -----------------------------------       
        LIGHTJoints.SetReference_Joint(true);
        LIGHTJoints.SetReference_Actuator(true);

        //////======================================================================================

        // -----------------------------------
        // Data Display
        // -----------------------------------
        if (CNT_display >= 200) {
            CNT_display = 0;
//            double LinForce_RKN = jCon->GetJointRefLinForce(RKN);
//            FILE_LOG(logINFO) << "Linear Force @ RKN : "<<LinForce_RKN;
//            double LinForce_LKN = jCon->GetJointRefLinForce(LKN);
//            FILE_LOG(logINFO) << "Linear Force @ LKN : "<<LinForce_LKN;
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

        if(save_Flag_forID == true){
            save_PutData_forID(save_Index_forID, ID_BNO);
            save_Index_forID++;
            if(save_Index_forID == (SAVENUM-1))
                save_Index_forID=0;
        }

        //////======================================================================================
//        rt_task_suspend(&rtTaskCon);

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


//==============================//
// Flag Thread
//==============================//
int CntFlagThread = 0;
void *RBFlagThread(void *)
{
//    rt_task_set_periodic(NULL, TM_NOW, 40*1000);

//    while(__IS_WORKING)
//    {
//        rt_task_wait_period(NULL);

////        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) { // Daemon : 250Hz, SeungHoon AL : 250Hz
////            jCon->JointUpdate();
////            if(CntFlagThread%2 == 1) {
////                rt_task_resume(&rtTaskCon);
////                CntFlagThread = 1;
////            } else {
//////                rt_task_resume(&rtTaskCon);
////            }
////            CntFlagThread++;
////        }

//        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
//            jCon->JointUpdate();
//            rt_task_resume(&rtTaskCon);
//        }
//    }
}


//////=====================================================================================

void ValveOpenmodeON(int BNO, int init_OPEN)
{
    sharedREF->ValveCtrlMode_Command[BNO] = ValveControlMode_Opening;
    jCon->SetRefValvePos(BNO, init_OPEN);
}

void PosCtrlmodeON(int BNO)
{
    double CurrentAngle = sharedSEN->ENCODER[BNO][0].CurrentAngle;
    jCon->SetJointRefAngle(BNO,CurrentAngle);
    jCon->SetJointRefAngVel(BNO, 0.0);
    jCon->SetJointRefTorque(BNO, 0.0);

    if (BNO==RAP||BNO==RAR) {
        sharedREF->ValveCtrlMode_Command[RAP] = ValveControlMode_PosOrFor;
        sharedREF->ValveCtrlMode_Command[RAR] = ValveControlMode_PosOrFor;
        sharedREF->PosOrFor_Selection[RAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RAR] = JointControlMode_Position;
    } else if (BNO==LAP||BNO==LAR) {
        sharedREF->ValveCtrlMode_Command[LAP] = ValveControlMode_PosOrFor;
        sharedREF->ValveCtrlMode_Command[LAR] = ValveControlMode_PosOrFor;
        sharedREF->PosOrFor_Selection[LAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LAR] = JointControlMode_Position;
    } else {
        sharedREF->ValveCtrlMode_Command[BNO] = ValveControlMode_PosOrFor;
        sharedREF->PosOrFor_Selection[BNO] = JointControlMode_Position;
    }
}

void TorCtrlmodeON(int BNO)
{
    if (BNO==RAP||BNO==RAR) {
        sharedREF->ValveCtrlMode_Command[RAP] = ValveControlMode_PosOrFor;
        sharedREF->ValveCtrlMode_Command[RAR] = ValveControlMode_PosOrFor;
        sharedREF->PosOrFor_Selection[RAP] = JointControlMode_Torque;
        sharedREF->PosOrFor_Selection[RAR] = JointControlMode_Torque;
    } else if (BNO==LAP||BNO==LAR) {
        sharedREF->ValveCtrlMode_Command[LAP] = ValveControlMode_PosOrFor;
        sharedREF->ValveCtrlMode_Command[LAR] = ValveControlMode_PosOrFor;
        sharedREF->PosOrFor_Selection[LAP] = JointControlMode_Torque;
        sharedREF->PosOrFor_Selection[LAR] = JointControlMode_Torque;
    } else {
        sharedREF->ValveCtrlMode_Command[BNO] = ValveControlMode_PosOrFor;
        sharedREF->PosOrFor_Selection[BNO] = JointControlMode_Torque;
    }
}

//////=====================================================================================

bool LIGHTJointSet::ValveOpenCtrl_SineWave()
{
    static int JNO;
    static double SINE_TIME;
    static double SINE_MAG;
    static double SINE_NUM;
    static double t = 0.0;

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        JNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Board Number
        SINE_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]/1000.0;
        SINE_MAG = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
        SINE_NUM = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

        if(JNO == 4||JNO == 5) {
            FILE_LOG(logWARNING) << "Target Board Number : Right Ankle (4,5)";
            ValveOpenmodeON(4,0);
            ValveOpenmodeON(5,0);
        } else if (JNO == 10||JNO == 11) {
            FILE_LOG(logWARNING) << "Target Board Number : Left Ankle (10,11)";
            ValveOpenmodeON(10,0);
            ValveOpenmodeON(11,0);
        } else {
            FILE_LOG(logWARNING) << "Target Board Number : " << JNO ;
            ValveOpenmodeON(JNO,0);
        }

        save_PutDataFlag = true;
        _OPERATION_STAGE++;

    } else if(_OPERATION_STAGE == 1) {

        double Ref_Opening = SINE_MAG*sin(2.0*3.1415*t/SINE_TIME);
        int OPENING = (int)Ref_Opening;

        if(JNO == 4||JNO == 5) {
            jCon->SetRefValvePos(4,OPENING);
            jCon->SetRefValvePos(5,OPENING);
        } else if (JNO == 10||JNO == 11) {
            jCon->SetRefValvePos(10,OPENING);
            jCon->SetRefValvePos(11,OPENING);
        } else {
            jCon->SetRefValvePos(JNO,OPENING);
        }

        if (t >= SINE_NUM*SINE_TIME) {
            t = 0.0;
            save_ActivateFlag = true;
            char filename[30] = "SineWaveOpening_";
            strcat(filename, JointNameList[JNO].toLocal8Bit().data());
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_ACTTEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }
}


bool LIGHTJointSet::ValveOpenCtrl_Jump() {

    static int OPENING_RHP;
    static int OPENING_RKN;
    static int OPENING_RAP;
    static int OPENING_LHP;
    static int OPENING_LKN;
    static int OPENING_LAP;
    static double JUMP_TIME, RETURN_TIME;
    static double CurrentAngle[6];
    static double DesiredAngle[6];
    static double t = 0.0;

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        if(t<=0.0) {
            JUMP_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            RETURN_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            OPENING_RHP = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            OPENING_RKN = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            OPENING_RAP = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
            OPENING_LHP = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];
            OPENING_LKN = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
            OPENING_LAP = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
            ValveOpenmodeON(RHP,OPENING_RHP);
            ValveOpenmodeON(RKN,OPENING_RKN);
            ValveOpenmodeON(RAP,OPENING_RAP); // RANK 1
            ValveOpenmodeON(RAR,OPENING_RAP); // RANK 2
            ValveOpenmodeON(LHP,OPENING_LHP);
            ValveOpenmodeON(LKN,OPENING_LKN);
            ValveOpenmodeON(LAP,OPENING_LAP); // LANK 1
            ValveOpenmodeON(LAR,OPENING_LAP); // LANK 2
            save_PutDataFlag = true;
        }

        if (t >= JUMP_TIME) {
            PosCtrlmodeON(RHP);
            PosCtrlmodeON(RKN);
            PosCtrlmodeON(RAP);
            PosCtrlmodeON(RAR);
            PosCtrlmodeON(LHP);
            PosCtrlmodeON(LKN);
            PosCtrlmodeON(LAP);
            PosCtrlmodeON(LAR);
            _OPERATION_STAGE++;
            t = 0.0;
        }

    } else if(_OPERATION_STAGE == 1) {
        if(t<=0.0) {
            CurrentAngle[0] = Qnow(RHP);
            CurrentAngle[1] = Qnow(RKN);
            CurrentAngle[2] = Qnow(RAP);
            CurrentAngle[3] = Qnow(LHP);
            CurrentAngle[4] = Qnow(LKN);
            CurrentAngle[5] = Qnow(LAP);
            DesiredAngle[0] = 0.0;
            DesiredAngle[1] = 0.0;
            DesiredAngle[2] = 40.0;
            DesiredAngle[3] = 0.0;
            DesiredAngle[4] = 0.0;
            DesiredAngle[5] = 40.0;
        }

        double Ref_Pos[6];
        double Ref_Vel[6];
        for(int i=0;i<6;i++) {
            Ref_Pos[i] = (DesiredAngle[i]-CurrentAngle[i])*(1.0-cos(PI*t/RETURN_TIME))/2.0 + CurrentAngle[i];
            Ref_Vel[i] = (DesiredAngle[i]-CurrentAngle[i])*(PI/RETURN_TIME)*sin(PI*t/RETURN_TIME)/2.0;
        }
        Qref(RHP) = Ref_Pos[0]*D2R;
        Qref(RKN) = Ref_Pos[1]*D2R;
        Qref(RAP) = Ref_Pos[2]*D2R;
        Qref(LHP) = Ref_Pos[3]*D2R;
        Qref(LKN) = Ref_Pos[4]*D2R;
        Qref(LAP) = Ref_Pos[5]*D2R;
        dQref(RHP) = Ref_Vel[0]*D2R;
        dQref(RKN) = Ref_Vel[1]*D2R;
        dQref(RAP) = Ref_Vel[2]*D2R;
        dQref(LHP) = Ref_Vel[3]*D2R;
        dQref(LKN) = Ref_Vel[4]*D2R;
        dQref(LAP) = Ref_Vel[5]*D2R;

        if (t >= RETURN_TIME) {
            _OPERATION_STAGE++;
            t = 0.0;
        }
    } else if(_OPERATION_STAGE == 2) {
        t = 0.0;
        save_ActivateFlag = true;
        strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, "JumpWithValveOpen");
        dQref(RHP) = 0.0;
        dQref(RKN) = 0.0;
        dQref(RAP) = 0.0;
        dQref(LHP) = 0.0;
        dQref(LKN) = 0.0;
        dQref(LAP) = 0.0;
        _OPERATION_FINISHED = true;
        _OPERATION_STAGE = 0;
        return true;
    }

    t += SYS_DT_ACTTEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }
}

bool LIGHTJointSet::PosCtrl_HomePose() {

    static VectorNd CurrentAngle = VectorNd::Zero(n_dof);
    static double IDX;

    if(_OPERATION_STAGE == 0) {
        for(int idx=0; idx<n_dof; idx++){
            CurrentAngle(idx) = sharedSEN->ENCODER[idx][0].CurrentRefAngle;
            PosCtrlmodeON(idx);
        }
        _OPERATION_STAGE++;
        IDX = 0.0;

    } else if(_OPERATION_STAGE == 1) {
        double MOVING_TIME = 3.0;
        for(int idx=0; idx<n_dof; idx++){
            double DesiredAngle = 0.0;
            double Ref_Pos = (CurrentAngle(idx)-DesiredAngle)*(1.0+cos(3.1415*IDX*SYS_DT_ACTTEST/MOVING_TIME))/2.0 + DesiredAngle;
            double Ref_Vel = -(CurrentAngle(idx)-DesiredAngle)*(3.1415/MOVING_TIME)*sin(3.1415*IDX*SYS_DT_ACTTEST/MOVING_TIME)/2.0;
            Qref(idx) = Ref_Pos*D2R;
            dQref(idx) = Ref_Vel*D2R;
        }

        if (IDX >= MOVING_TIME*SYS_FREQ_ACTTEST) {
            for(int idx=0; idx<n_dof; idx++) dQref(idx) = 0.0;
            IDX = 0.0;
            _OPERATION_STAGE++;
        } else {
            IDX=IDX+1.0;
        }
    } else if(_OPERATION_STAGE == 2) {
        _OPERATION_FINISHED = true;
        _OPERATION_STAGE = 0;
    }

    if(_OPERATION_FINISHED) {
        return true;
    } else { return false; }
}

bool LIGHTJointSet::PosCtrl_WalkReadyPose() {

    static VectorNd CurrentAngle = VectorNd::Zero(n_dof);
    static VectorNd DesiredAngle = VectorNd::Zero(n_dof);
    DesiredAngle << 0.0, 0.0, 60.0, -55.0, 42.0, 0.0,
                    0.0, 0.0, 60.0, -55.0, 42.0, 0.0,
                    0.0;
    static double IDX;

    if(_OPERATION_STAGE == 0) {
        for(int idx=0; idx<n_dof; idx++){
            CurrentAngle(idx) = sharedSEN->ENCODER[idx][0].CurrentRefAngle;
            PosCtrlmodeON(idx);
        }
        _OPERATION_STAGE++;
        IDX = 0.0;

    } else if(_OPERATION_STAGE == 1) {
        double MOVING_TIME = 3.0;
        for(int idx=0; idx<n_dof; idx++){
            double Ref_Pos = (CurrentAngle(idx)-DesiredAngle(idx))*(1.0+cos(3.1415*IDX*SYS_DT_ACTTEST/MOVING_TIME))/2.0 + DesiredAngle(idx);
            double Ref_Vel = -(CurrentAngle(idx)-DesiredAngle(idx))*(3.1415/MOVING_TIME)*sin(3.1415*IDX*SYS_DT_ACTTEST/MOVING_TIME)/2.0;
            Qref(idx) = Ref_Pos*D2R;
            dQref(idx) = Ref_Vel*D2R;
        }

        if (IDX >= MOVING_TIME*SYS_FREQ_ACTTEST) {
            for(int idx=0; idx<n_dof; idx++) dQref(idx) = 0.0;
            IDX = 0.0;
            _OPERATION_STAGE++;
        } else {
            IDX=IDX+1.0;
        }
    } else if(_OPERATION_STAGE == 2) {
        _OPERATION_FINISHED = true;
        _OPERATION_STAGE = 0;
    }

    if(_OPERATION_FINISHED) {
        return true;
    } else { return false; }
}


bool LIGHTJointSet::PosCtrl_GOTO_Pose() {

    static int JNO;
    static int OPENTYPE;
    static double DesiredAngle;
    static double CurrentAngle;
    static double MOVING_TIME;
    static double IDX;

    if(_OPERATION_STAGE == 0) {
        JNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Joint Number

        DesiredAngle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
        CurrentAngle = sharedSEN->ENCODER[JNO][0].CurrentRefAngle;
        MOVING_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; //sec
        FILE_LOG(logSUCCESS) << "Desired Angle : " << DesiredAngle;
        FILE_LOG(logSUCCESS) << "Target Joint Number : " << JNO;

        PosCtrlmodeON(JNO);
        _OPERATION_STAGE++;
    } else if(_OPERATION_STAGE == 1) {
        double Ref_Pos = (CurrentAngle-DesiredAngle)*(1.0+cos(3.1415*IDX*SYS_DT_ACTTEST/MOVING_TIME))/2.0 + DesiredAngle;
        double Ref_Vel = -(CurrentAngle-DesiredAngle)*(3.1415/MOVING_TIME)*sin(3.1415*IDX*SYS_DT_ACTTEST/MOVING_TIME)/2.0;
        Qref(JNO) = Ref_Pos*D2R;
        dQref(JNO) = Ref_Vel*D2R;
        if (IDX >= MOVING_TIME*SYS_FREQ_ACTTEST) {
            dQref(JNO) = 0.0;
            IDX = 0.0;
            _OPERATION_STAGE++;
        } IDX=IDX+1.0;
    } else if(_OPERATION_STAGE == 2) {
        _OPERATION_FINISHED = true;
        _OPERATION_STAGE = 0;
    }

    if(_OPERATION_FINISHED) {
        return true;
    } else { return false; }
}

bool LIGHTJointSet::PosCtrl_Random_Motion() {

    static int JNO;
    static int OPENTYPE;
    static double CurrentAngle;
    static double MOVING_TIME;
    static double IDX;
    static double a[7];

    if(_OPERATION_STAGE == 0) {
        srand((unsigned int)time(NULL));
        int max_freq = 4; // [Hz], up to 6
        for(int idx = 1; idx<max_freq+1; idx++) {
            int temp_random = rand();
            temp_random = temp_random%2001 - 1000; //(-1000~1000)
            a[idx] = (double)temp_random/100.0;
            a[0] = a[0] - a[idx];
        }

        JNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Board Number
        OPENTYPE = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]; // Opening Type Select
        MOVING_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; //sec
        CurrentAngle = sharedSEN->ENCODER[JNO][0].CurrentRefAngle;
        FILE_LOG(logSUCCESS) << "Target Joint Number : " << JNO;

        PosCtrlmodeON(JNO);
        _OPERATION_STAGE++;


    } else if(_OPERATION_STAGE == 1) {

        double Ref_Pos = a[0]+CurrentAngle;
        double Ref_Vel = 0.0;

        for(int idx = 1; idx<7; idx++) {
            double k = (double)idx;
            Ref_Pos += a[idx]*cos(2.0*3.1415*k*SYS_DT_ACTTEST*IDX);
            Ref_Vel += -a[idx]*2.0*3.1415*k*sin(2*3.1415*k*SYS_DT_ACTTEST*IDX);
        }
        Qref(JNO) = Ref_Pos*D2R;
        dQref(JNO) = Ref_Vel*D2R;

        if (IDX >= MOVING_TIME*SYS_FREQ_ACTTEST) {
            dQref(JNO) = 0.0;      // Velocity Reference Reset
            IDX = 0.0;
            _OPERATION_STAGE++;
        } IDX=IDX+1.0;

    } else if(_OPERATION_STAGE == 2) {
        _OPERATION_FINISHED = true;
        _OPERATION_STAGE = 0;
    }

    if(_OPERATION_FINISHED) {
        return true;
    } else { return false; }
}

bool LIGHTJointSet::PosCtrl_Sine_Motion() {

    static int JNO;
    static int OPENTYPE;
    static double SINE_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
    static double SINE_MAG = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
    static double SINE_NUM = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
    static double CurrentAngle;
    static double t = 0.0;

    static double dT_window = sharedREF->dT_PrevPump;

    static double t_init = (double)MAX_PREVIEW*dT_window;

    if(_OPERATION_STAGE == 0) { // Parameters Setting
        if(t==0) {
            JNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]; // Board Number
            OPENTYPE = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]; // Opening Type Select
            SINE_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            SINE_MAG = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            SINE_NUM = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            CurrentAngle = sharedSEN->ENCODER[JNO][0].CurrentRefAngle;
            FILE_LOG(logSUCCESS) << "Target Joint Number : " << JNO;
            PosCtrlmodeON(JNO);
        }


        for(int i=0;i<=MAX_PREVIEW;i++) {
            double t_view = t + SYS_DT_ACTTEST + dT_window*(double)(i);
            if(t_view < t_init) {
                sharedREF->ActFlowrateReference_Future[i][0] = 0.0;
            } else {
                double Ref_Vel = D2R*fabs((2.0*3.1415/SINE_TIME)*SINE_MAG*sin(2.0*3.1415*(t_view-t_init)/SINE_TIME)/2.0);
                double Area_HipYaw  = 0.02525*0.0095*0.020*2; //rotary, rA = m^3
                sharedREF->ActFlowrateReference_Future[i][0] = Area_HipYaw*Ref_Vel*60000.0;
            }
        }

        if (t >= t_init) {
            save_PutDataFlag = true;
            _OPERATION_STAGE++;
            t = 0.0;
        }

    } else if(_OPERATION_STAGE == 1) {
        double Ref_Pos = SINE_MAG*(1.0-cos(2.0*3.1415*t/SINE_TIME))/2.0+CurrentAngle;
        double Ref_Vel = (2.0*3.1415/SINE_TIME)*SINE_MAG*sin(2.0*3.1415*t/SINE_TIME)/2.0;
        Qref(JNO) = Ref_Pos*D2R;
        dQref(JNO) = Ref_Vel*D2R;

        double dT_window = sharedREF->dT_PrevPump;
        for(int i=0;i<=MAX_PREVIEW;i++) {
            double t_view = t + SYS_DT_ACTTEST + dT_window*(double)(i);
            if(t_view < SINE_NUM*SINE_TIME) {
                double Ref_Vel = D2R*fabs((2.0*3.1415/SINE_TIME)*SINE_MAG*sin(2.0*3.1415*t_view/SINE_TIME)/2.0);
                double Area_HipYaw  = 0.02525*0.0095*0.020*2; //rotary, rA = m^3
                sharedREF->ActFlowrateReference_Future[i][0] = Area_HipYaw*Ref_Vel*60000.0;
            } else {
                sharedREF->ActFlowrateReference_Future[i][0] = 0.0;
            }
        }

        if (t >= SINE_NUM*SINE_TIME) {
            dQref(JNO) = 0.0;  // Velocity Reference Reset
            t = 0.0;
            save_ActivateFlag = true;
            char filename[30] = "SineWaveMotion_";
            strcat(filename, JointNameList[JNO].toLocal8Bit().data());
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);

            _OPERATION_FINISHED = true;
            _OPERATION_STAGE = 0;
            return true;
        }
    }

    t += SYS_DT_ACTTEST;

    if(_OPERATION_FINISHED) {
        t = 0.0;
        return true;
    } else { return false; }
}

bool LIGHTJointSet::TorCtrl_ConstantTorque(double DesiredTorque,
                                           double Kp_Comp, double Ki_Comp,
                                           bool _StopFlag)
{
    static double _t_now = 0.0;
    static int JNO;
    static double errorsum_Mx = 0.0, errorsum_My = 0.0;

    if(_OPERATION_STAGE == 0) {
        if(_t_now == 0.0) {
            FILE_LOG(logSUCCESS) << "Torque Control Start!";

            JNO = _OPERATION_PARA_INT[0]; // Joint Number
            FILE_LOG(logSUCCESS) << "Desired Torque : " << DesiredTorque;
            FILE_LOG(logSUCCESS) << "Target Joint Number : " << JNO;
            TorCtrlmodeON(JNO);
            Tref(JNO) = DesiredTorque;
        }
        if(_t_now > 3.0) {
            _OPERATION_STAGE++;
            _t_now = 0.0;
            errorsum_Mx = 0.0;
            errorsum_My = 0.0;
            save_PutDataFlag = true;
        } else {
            _t_now += SYS_DT_ACTTEST;
        }
    } else if(_OPERATION_STAGE == 1) {
        Tref(JNO) = DesiredTorque;
        if(JNO==RAP||JNO==RAR) {
            double error_Mx = Tref(RAR) - sharedSEN->FT[0].Mx;
            double error_My = Tref(RAP) - sharedSEN->FT[0].My;
            errorsum_Mx += error_Mx*SYS_DT_ACTTEST;
            errorsum_My += error_My*SYS_DT_ACTTEST;
            dQref_CompMx_RANK = Kp_Comp*error_Mx;
            dQref_CompMy_RANK = Kp_Comp*error_My;
            Tref_CompMx_RANK = Ki_Comp*errorsum_Mx;
            Tref_CompMy_RANK = Ki_Comp*errorsum_My;
            save_Vector(0) = Tref(RAR);
            save_Vector(1) = Tref(RAP);
            save_Vector(2) = Tnow(RAR);
            save_Vector(3) = Tnow(RAP);
        }
        if(JNO==LAP||JNO==LAR) {
            double error_Mx = Tref(LAR) - sharedSEN->FT[1].Mx;
            double error_My = Tref(LAP) - sharedSEN->FT[1].My;
            errorsum_Mx += error_Mx*SYS_DT_ACTTEST;
            errorsum_My += error_My*SYS_DT_ACTTEST;
            dQref_CompMx_LANK = Kp_Comp*error_Mx;
            dQref_CompMy_LANK = Kp_Comp*error_My;
            Tref_CompMx_LANK = Ki_Comp*errorsum_Mx;
            Tref_CompMy_LANK = Ki_Comp*errorsum_My;
            save_Vector(4) = Tref(LAR);
            save_Vector(5) = Tref(LAP);
            save_Vector(6) = Tnow(LAR);
            save_Vector(7) = Tnow(LAP);
        }

        if(_StopFlag) {
            _OPERATION_STAGE++;
            dQref_CompMx_RANK = 0.0;
            dQref_CompMy_RANK = 0.0;
            dQref_CompMx_LANK = 0.0;
            dQref_CompMy_LANK = 0.0;
            char filename[30] = "JointTorqueControl";
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, filename);
            save_ActivateFlag = true;
        }
    } else if(_OPERATION_STAGE == 2) {
        FILE_LOG(logERROR) << "Torque Control End!";

        LIGHTJoints.Tref(JNO) = 0.0;
        PosCtrlmodeON(JNO);

        _OPERATION_FINISHED = true;
        _OPERATION_STAGE = 0;
    }

    if(_OPERATION_FINISHED) {
        return true;
    } else { return false; }
}


//////=====================================================================================

enum Variable_Ps_STAGESET{
    Variable_Ps_PARAMETER_SETTING = 0,
    Variable_Ps_INITIALIZE,
    Variable_Ps_PROCESS,
    Variable_Ps_FINISH,
};
int Variable_Ps_CurrentStage = Variable_Ps_PARAMETER_SETTING;

bool LIGHTJointSet::Variable_SupplyPressure_Test(bool _ONOFF,
                                                 double _MAG, double _PER, int _N,
                                                 double _LOAD)
{
    static double L = 0.40;
    static double M = _LOAD;

    static double q[MAX_PREVIEW+1] = {0.0};
    static double dq[MAX_PREVIEW+1] = {0.0};
    static double ddq[MAX_PREVIEW+1] = {0.0};

    static double q_init = 0.0;
    static double q_mag = 0.0;

    static double T = 10.0;
    static int    N = _N;

    static double dT_window = sharedREF->dT_PrevPump;

    static double t = 0.0;
    static double t_ready = 3.0;
    static double t_init = 2.0;
    static double t_term = 2.0;
    static int n = 0;

    static bool FlagForcedStop = false;
    if(!_ONOFF && !FlagForcedStop) { // all stop
        FILE_LOG(logERROR) << "Variable Ps Task, Forced Stopped!";
        FlagForcedStop = true;
        Variable_Ps_CurrentStage = Variable_Ps_FINISH;
        t = 0.0;
        n = 0;
    }

    switch(Variable_Ps_CurrentStage) {
    case Variable_Ps_PARAMETER_SETTING: {
        if(t == 0.0) {
            q_init = Qref(0);
            q_mag = _MAG*D2R;
            T = _PER;
            N = _N;
            M = _LOAD;
            dT_window = sharedREF->dT_PrevPump;
            FILE_LOG(logWARNING) << "Variable Ps Task, Parameters Setting...";
            PosCtrlmodeON(0);
        }

        for(int i=0;i<=MAX_PREVIEW;i++) {
            double t_view = SYS_DT_ACTTEST + dT_window*(double)i;
            if(t_view < t_ready) {
                q[i] = q_init;
                dq[i] = 0.0;
                ddq[i] = 0.0;
            } else {
                double q_next, dq_next, ddq_next;
                fifth_trajectory_oneaxis(t_init, (t+t_view)-t_ready,
                                         q_init, 0.0, 0.0,
                                         q_init + q_mag, 0.0, -q_mag*(2.0*PI/T)*(2.0*PI/T)/2.0,
                                         q_next, dq_next, ddq_next);
                q[i] = q_next;
                dq[i] = dq_next;
                ddq[i] = ddq_next;
            }
        }

        if(t>=t_ready) {
            Variable_Ps_CurrentStage = Variable_Ps_INITIALIZE;
            t = 0.0;
            FILE_LOG(logWARNING) << "Ready.";
        }
        break;
    }
    case Variable_Ps_INITIALIZE: {

        for(int i=0;i<=MAX_PREVIEW;i++) {
            double t_view = SYS_DT_ACTTEST + dT_window*(double)i;
            if(t_view < t_init) {
                double q_next, dq_next, ddq_next;
                fifth_trajectory_oneaxis(t_init-t, t_view,
                                         q[0], dq[0], ddq[0],
                                         q_init + q_mag, 0.0, -q_mag*(2.0*PI/T)*(2.0*PI/T)/2.0,
                                         q_next, dq_next, ddq_next);
                q[i] = q_next;
                dq[i] = dq_next;
                ddq[i] = ddq_next;
            } else {
                q[i] = q_init+q_mag*(1.0+cos(2.0*PI/T*(t-t_init+t_view)))/2.0;
                dq[i] = -q_mag*(2.0*PI/T)*sin(2.0*PI/T*(t-t_init+t_view))/2.0;
                ddq[i] = -q_mag*(2.0*PI/T)*(2.0*PI/T)*cos(2.0*PI/T*(t-t_init+t_view))/2.0;
            }
        }

        if(t>=t_init) {
            Variable_Ps_CurrentStage = Variable_Ps_PROCESS;
            t = 0.0;
            FILE_LOG(logSUCCESS) << "Start!";
        }
        break;
    }
    case Variable_Ps_PROCESS: {

        for(int i=0;i<=MAX_PREVIEW;i++) {
            double t_view = SYS_DT_ACTTEST + dT_window*(double)i;
            if(t_view < (double)N*T) {
                q[i] = q_init+q_mag*(1.0+cos(2.0*PI/T*(t+t_view)))/2.0;
                dq[i] = -q_mag*(2.0*PI/T)*sin(2.0*PI/T*(t+t_view))/2.0;
                ddq[i] = -q_mag*(2.0*PI/T)*(2.0*PI/T)*cos(2.0*PI/T*(t+t_view))/2.0;
            } else {
                double q_next, dq_next, ddq_next;
                fifth_trajectory_oneaxis(t_term, (t+t_view)-(double)N*T,
                                         q_init+q_mag, 0.0, -q_mag*(2.0*PI/T)*(2.0*PI/T),
                                         q_init, 0.0, 0.0,
                                         q_next, dq_next, ddq_next);
                q[i] = q_next;
                dq[i] = dq_next;
                ddq[i] = ddq_next;
            }
        }

        if(t>=(double)N*T) {
            FILE_LOG(logWARNING) << "Variable Ps Task, terminated...";
            Variable_Ps_CurrentStage = Variable_Ps_FINISH;
            t = 0.0;
            n = 0;
        }
        break;
    }
    case Variable_Ps_FINISH: {

        for(int i=0;i<=MAX_PREVIEW;i++) {
            double t_view = SYS_DT_ACTTEST + dT_window*(double)i;
            if(t_view < t_term) {
                double q_next, dq_next, ddq_next;
                fifth_trajectory_oneaxis(t_term-t, t_view,
                                         q[0], dq[0], ddq[0],
                                         q_init, 0.0, 0.0,
                                         q_next, dq_next, ddq_next);
                q[i] = q_next;
                dq[i] = dq_next;
                ddq[i] = ddq_next;
            } else {
                q[i] = q_init;
                dq[i] = 0.0;
                ddq[i] = 0.0;
            }
        }

        if(t>=t_term) {
            FILE_LOG(logERROR) << "Variable Ps Task, Done!";
            for(int i=0;i<=MAX_PREVIEW;i++) {
                q[i] = q_init;
                dq[i] = 0.0;
                ddq[i] = 0.0;
            }
            Variable_Ps_CurrentStage = Variable_Ps_PARAMETER_SETTING;
            t = 0.0;
            FlagForcedStop = false;
            _OPERATION_FINISHED = true;
        }
        break;
    }
    default:
        break;
    }

    Qref(0) = q[0];
    dQref(0) = dq[0];
    ddQref(0) = ddq[0];
    Tref(0) = M*L*(L*ddQref(0)-g_const*sin(Qref(0)));

    for(int i=0;i<=MAX_PREVIEW;i++) {
        double Tf = M*L*(L*ddq[i]+g_const*cos(q[i])); // future torque
        double dthf = dq[i]; // future angular velocity

        // Derive Minimum Supply Pressure
        double rA = 9595.0*1e-3; // mm^2*m
        double Ps_load = fabs(Tf/rA*10.0); // MPa >> bar
        double Qact_des = fabs(dthf*rA/1000.0*60.0); // rad/s >> L/min

        double Kv = 6.9/sqrt(70.0);
        double Ps_des;
        if(Qact_des >= 0 ) {
            Ps_des = Ps_load + 2.0*(Qact_des/Kv)*(Qact_des/Kv);
        } else {
            Ps_des = -Ps_load + 2.0*(Qact_des/Kv)*(Qact_des/Kv);
        }

        sharedREF->LoadPressureReference_Future[i][0] = Ps_des;
        sharedREF->ActFlowrateReference_Future[i][0] = Qact_des;
    }

    FILE_LOG(logDEBUG) << "Ps_ini : " << sharedREF->LoadPressureReference_Future[0][0];
//    FILE_LOG(logDEBUG) << "Ps_fin : " << sharedREF->LoadPressureReference_Future[MAX_PREVIEW][0];
//    FILE_LOG(logDEBUG) << " ==================== ";
//    FILE_LOG(logDEBUG) << "q_ini : " << q[0]*R2D;
//    FILE_LOG(logDEBUG) << "q_fin : " << q[MAX_PREVIEW]*R2D;
//    FILE_LOG(logDEBUG) << " ==================== ";

    if(_OPERATION_FINISHED) {
        return true;
    }

    t += SYS_DT_ACTTEST;

    return false;
}


//////=====================================================================================

void Generate_PumpPressureRef(VectorNd Q, VectorNd dQ, VectorNd T, VectorNd &Qact, VectorNd &Pload)
{

    const double Sa[12] = {5100.0*1e-3, 9595.0*1e-3, 236.0, 491.0, 113.0, 113.0,
                           5100.0*1e-3, 9595.0*1e-3, 236.0, 491.0, 113.0, 113.0};
    const double Sb[12] = {5100.0*1e-3, 9595.0*1e-3, 236.0, 236.0, 63.0, 63.0,
                           5100.0*1e-3, 9595.0*1e-3, 236.0, 236.0, 63.0, 63.0};

//    const double Sa[12] = {9595.0*1e-3, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0,
//                           1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
//    const double Sb[12] = {9595.0*1e-3, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0,
//                           1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};

    double alpha[12];
    for(int i=0;i<12;i++) {
        alpha[i] = Sa[i]/Sb[i];

        double _Qref = Q(i);
        double _dQref = dQ(i);
        double _Tref = T(i);
        if (i==RHP||i==LHP) {

//            double _dXref = LIGHTJoints.AngularVel2CylinderVel_Hip(_Qref, _dQref); // cylinder velocity [m/s]
//            double _Fref = LIGHTJoints.Torque2CylinderForce_Hip(_Qref, _Tref); // cylinder force [F]
            double _dXref = 0.0; // cylinder velocity [m/s]
            double _Fref = 0.0; // cylinder force [F]

            if(_dXref>0) {
                Qact(i) = _dXref*Sa[i]/1000.0*60.0;
            } else {
                Qact(i) = _dXref*Sb[i]/1000.0*60.0;
            }
            Pload(i) = _Fref/Sb[i]*10.0;

        } else if (i==RKN||i==LKN) {

//            double _dXref = LIGHTJoints.AngularVel2CylinderVel_Knee(_Qref, _dQref); // cylinder velocity [m/s]
//            double _Fref = LIGHTJoints.Torque2CylinderForce_Knee(_Qref, _Tref); // cylinder force [F]
            double _dXref = 0.0; // cylinder velocity [m/s]
            double _Fref = 0.0; // cylinder force [F]

            if(_dXref>0) {
                Qact(i) = _dXref*Sa[i]/1000.0*60.0;
            } else {
                Qact(i) = _dXref*Sb[i]/1000.0*60.0;
            }
            Pload(i) = _Fref/Sb[i]*10.0;

        } else if (i==RAP||i==LAP) {

            double _Q2ref = Q(i+1);
            double _dQ2ref = dQ(i+1);
            double _T2ref = T(i+1);

//            VectorNd _dXref = LIGHTJoints.AngularVel2CylinderVel_Ankle_NewSW(_Qref, _Q2ref, _dQref, _dQ2ref); // cylinder velocity [m/s]
//            VectorNd _Fref = LIGHTJoints.Torque2CylinderForce_Ankle_NewSW(_Qref, _Q2ref, _Tref, _T2ref); // cylinder force [N]
            VectorNd _dXref = VectorNd::Zero(2); // cylinder velocity [m/s]
            VectorNd _Fref = VectorNd::Zero(2); // cylinder force [F]

            if(_dXref(0)>0) {
                Qact(i) = _dXref(0)*Sa[i]/1000.0*60.0;
            } else {
                Qact(i) = _dXref(0)*Sb[i]/1000.0*60.0;
            }
            if(_dXref(1)>0) {
                Qact(i+1) = _dXref(1)*Sa[i+1]/1000.0*60.0;
            } else {
                Qact(i+1) = _dXref(1)*Sb[i+1]/1000.0*60.0;
            }
            Pload(i) = _Fref(0)/Sb[i]*10.0;
            Pload(i+1) = _Fref(1)/Sb[i+1]*10.0;

            i++; // Skip RAR or LAR

        } else { // Rotary Actuator

            Qact(i) = _dQref*Sa[i]/1000.0*60.0;
            Pload(i) = _Tref/Sa[i]*10.0;

        }
    }
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
// Saving function
//==============================//

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = SYS_DT_ACTTEST;

    // RIGHT HIP YAW (Board 0)
    save_Buf[1][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentRefAngle;
    save_Buf[2][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentAngle;
    save_Buf[3][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentRefTorque;
    save_Buf[4][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentTorque;
    save_Buf[5][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentValvePos;

    // RIGHT HIP ROLL (Board 1)
    save_Buf[6][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentRefAngle;
    save_Buf[7][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentAngle;
    save_Buf[8][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentRefTorque;
    save_Buf[9][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentTorque;
    save_Buf[10][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentValvePos;

    // RIGHT HIP PITCH (Board 2)
    save_Buf[11][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefAngle;
    save_Buf[12][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentAngle;
    save_Buf[13][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefActPos;
    save_Buf[14][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentActPos;
    save_Buf[15][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefActForce;
    save_Buf[16][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentActForce;
    save_Buf[17][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefTorque;
    save_Buf[18][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentTorque;
    save_Buf[19][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefValvePos;
    save_Buf[20][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentValvePos;

    // RIGHT KNEE (Board 3)
    save_Buf[21][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefAngle;
    save_Buf[22][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentAngle;
    save_Buf[23][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefActPos;
    save_Buf[24][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentActPos;
    save_Buf[25][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefActForce;
    save_Buf[26][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentActForce;
    save_Buf[27][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefTorque;
    save_Buf[28][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentTorque;
    save_Buf[29][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefValvePos;
    save_Buf[30][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentValvePos;

    // RIGHT ANKLE1 (Board 4)
    save_Buf[31][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefAngle;
    save_Buf[32][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentAngle;
    save_Buf[33][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefActPos;
    save_Buf[34][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentActPos;
    save_Buf[35][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefActForce;
    save_Buf[36][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentActForce;
    save_Buf[37][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefTorque;
    save_Buf[38][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentTorque;
    save_Buf[39][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefValvePos;
    save_Buf[40][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentValvePos;

    // RIGHT ANKLE2 (Board 5)
    save_Buf[41][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefAngle;
    save_Buf[42][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentAngle;
    save_Buf[43][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefActPos;
    save_Buf[44][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentActPos;
    save_Buf[45][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefActForce;
    save_Buf[46][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentActForce;
    save_Buf[47][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefTorque;
    save_Buf[48][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentTorque;
    save_Buf[49][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefValvePos;
    save_Buf[50][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentValvePos;

    // LEFT HIP YAW (Board 6)
    save_Buf[51][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentRefAngle;
    save_Buf[52][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentAngle;
    save_Buf[53][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentRefTorque;
    save_Buf[54][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentTorque;
    save_Buf[55][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentValvePos;

    // LEFT HIP ROLL (Board 7)
    save_Buf[56][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentRefAngle;
    save_Buf[57][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentAngle;
    save_Buf[58][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentRefTorque;
    save_Buf[59][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentTorque;
    save_Buf[60][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentValvePos;

    // LEFT HIP PITCH (Board 8)
    save_Buf[61][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefAngle;
    save_Buf[62][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentAngle;
    save_Buf[63][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefActPos;
    save_Buf[64][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentActPos;
    save_Buf[65][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefActForce;
    save_Buf[66][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentActForce;
    save_Buf[67][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefTorque;
    save_Buf[68][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentTorque;
    save_Buf[69][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefValvePos;
    save_Buf[70][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentValvePos;

    // LEFT KNEE (Board 9)
    save_Buf[71][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefAngle;
    save_Buf[72][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentAngle;
    save_Buf[73][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefActPos;
    save_Buf[74][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentActPos;
    save_Buf[75][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefActForce;
    save_Buf[76][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentActForce;
    save_Buf[77][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefTorque;
    save_Buf[78][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentTorque;
    save_Buf[79][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefValvePos;
    save_Buf[80][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentValvePos;

    // LEFT ANKLE1 (Board 10)
    save_Buf[81][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefAngle;
    save_Buf[82][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentAngle;
    save_Buf[83][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefActPos;
    save_Buf[84][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentActPos;
    save_Buf[85][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefActForce;
    save_Buf[86][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentActForce;
    save_Buf[87][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefTorque;
    save_Buf[88][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentTorque;
    save_Buf[89][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefValvePos;
    save_Buf[90][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentValvePos;

    // LEFT ANKLE2 (Board 11)
    save_Buf[91][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefAngle;
    save_Buf[92][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentAngle;
    save_Buf[93][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefActPos;
    save_Buf[94][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentActPos;
    save_Buf[95][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefActForce;
    save_Buf[96][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentActForce;
    save_Buf[97][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefTorque;
    save_Buf[98][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentTorque;
    save_Buf[99][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefValvePos;
    save_Buf[100][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentValvePos;

    // PUMP
    save_Buf[101][cur_Index] = sharedREF->PumpPressureReference[0];
    save_Buf[102][cur_Index] = sharedSEN->PUMP[0].CurrentPressure;
    save_Buf[103][cur_Index] = sharedSEN->PUMP[0].CurrentRefVelocity;
    save_Buf[104][cur_Index] = sharedSEN->PUMP[0].CurrentVelocity;

    save_Buf[105][cur_Index] = sharedSEN->FT[0].Mx;
    save_Buf[106][cur_Index] = sharedSEN->FT[0].My;
    save_Buf[107][cur_Index] = sharedSEN->FT[0].Fz;
    save_Buf[108][cur_Index] = sharedSEN->FT[1].Mx;
    save_Buf[109][cur_Index] = sharedSEN->FT[1].My;
    save_Buf[110][cur_Index] = sharedSEN->FT[1].Fz;

    save_Buf[111][cur_Index] = save_Vector(0);
    save_Buf[112][cur_Index] = save_Vector(1);
    save_Buf[113][cur_Index] = save_Vector(2);
    save_Buf[114][cur_Index] = save_Vector(3);
    save_Buf[115][cur_Index] = save_Vector(4);
    save_Buf[116][cur_Index] = save_Vector(5);
    save_Buf[117][cur_Index] = save_Vector(6);
    save_Buf[118][cur_Index] = save_Vector(7);
    save_Buf[119][cur_Index] = save_Vector(8);
    save_Buf[120][cur_Index] = save_Vector(9);
    save_Buf[121][cur_Index] = save_Vector(10);
    save_Buf[122][cur_Index] = save_Vector(11);
}

void save_File(char *filecomment)
{
    int n_col = save_Index;
    save_Index = 0;

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "DataLog/SH_AL_Data/%Y%m%d_%X");
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
