#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#include <QSettings>

//#include "RBCAN.h"
#include "RBRawLAN.h"
#include "RBDataBase.h"
#include "../../share/Headers/JointInformation.h"

#include "RBProcessManager.h"
#include "RBSPI2CAN.h"

#include "HydraulicActuatorDataConverting.h"
#include "HydraulicActuatorController.h"
#include "HydraulicPumpController.h"
#include "RBFTSensor.h"
#include "RBIMUSensor.h"

#include "RBThread.h"
#include "IKF.h"

//#include "Eigen/Dense"
//#include "rbdl/rbdl.h"

using namespace std;

QString     settingFile;

// Basic --------
int     IS_WORKING = false;
int     IS_CHILD = false;
int     IS_CAN_READY = false;
int     IS_RS232_OK = false;

int     NO_RESPONSE_CNT = 0;

pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
//pRBCAN                  canHandler;
pRBLAN                  lanHandler;
RBProcessManager        *pmHandler;

// Initialize --------
int     RBCore_Initialize();
int     RBCore_SMInitialize();
int     RBCore_DBInitialize();
int     RBCore_CANInitialize();
int     RBCore_LANInitialize();
int     RBCore_ThreadInitialize();
int     RBCore_PMInitialize();
int     RBCore_Termination();

void    *RBCore_RTThreadCon(void *);
//void    *RBCore_NRTThreadCon(void *);

ulong   rtTaskCon;
ulong   nrtTaskCon;

// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_VC           RBDataBase::_DB_VC[MAX_VC];
DB_PC           RBDataBase::_DB_PC[MAX_PC];
DB_FT           RBDataBase::_DB_FT[MAX_FT];
DB_IMU          RBDataBase::_DB_IMU[MAX_IMU];
DB_AL           RBDataBase::_DB_AL[MAX_AL];

int     _VERSION;
int     _NO_OF_AL;
int     _NO_OF_COMM_CH;
int     _NO_OF_VC;
int     _NO_OF_PC;
int     _NO_OF_FT;
int     _NO_OF_IMU;


// Devices --------
ValveController             _DEV_VC[MAX_VC];
PumpController              _DEV_PC[MAX_PC];
RBIMUSensor                 _DEV_IMU[MAX_IMU];
RBFTSensor                  _DEV_FT[MAX_FT];

int _CANOUT_ENABLED = false;
int _SENSOR_ENABLED = false;


// Reference Buffer ------
double  finalJointPosRef[MOTOR_2CH];
double  finalJointVelRef[MOTOR_2CH];
double  finalJointFTRef[MOTOR_2CH];
double  finalValvePosRef[MOTOR_2CH];
double  finalPWMRef[MOTOR_2CH];
double  finalPumpVelocityRef = 0.0;

// Save Function ------
#define SAVEKIND    30
#define SAVENUM     100000

bool            save_Flag = false;
unsigned int    save_Index = 0;
float           save_Buf[SAVEKIND][SAVENUM];
void            save_PutData(unsigned int cur_Index);
void            save_File();

void            SetWaitTime_us(int T);
void            WaitDisplay_us(void);
long            WaitTime_us = 0; // umsec

int _ALCommandCnt[MAX_AL] = {0,};

long _ThreadCnt = 0;

bool StatusReadFlag[MAX_VC] = {0,};
bool ErrorClearStart = false;

// Data Read Functions
void    ReadBoardInfo();
void    ReadActuatorData();
void    ReadValvePosnPWM();
void    ReadValvePressure();
void    ReadValveOtherData();
void    ReadValveAlart();
void    ReadPumpData();
void    ReadNewIMU();
void    ReadFT();
void    ReadSimulationData();

// Reference Write Functions
void    WriteValveReference();
void    WritePumpReference();

// Command Functions
void    RBCMD_CAN_Check();
void    RBCMD_CAN_Channel_Arrange();

void    RBCMD_InitFETOnOff();
void    RBCMD_EncoderZero();
void    RBCMD_MotionRefOnOff();

extern void    RBCMD_ValveControllerRequestOnOff(void);
extern void    RBCMD_AskValveControllerParameters(int _BN);
extern void    RBCMD_SetValveControllerParameters(int _BN, HCB_INFO H);
extern void    RBCMD_ReadnSaveValveControllerParameters(void);
extern void    RBCMD_LoadnSetValveControllerParameters(void);

void    RBCMD_EncZero(void);
void    RBCMD_FindHome(void);
void    FindHomeOperation(int _BN);

extern void CylinderPos2Angle_Ankle(double theta, double phi, double x_R, double x_L,
                                    double& new_theta, double& new_phi);
extern void CylinderVel2AngularVel_Ankle(double theta, double phi, double dx_R, double dx_L,
                                         double& dtheta, double& dphi);
extern void CylinderForce2Torque_Ankle(double theta, double phi, double F_R, double F_L,
                                       double& T_pitch, double& T_roll);

extern void CylinderPos2Angle_Ankle_NewSW(double theta, double phi, double x_R, double x_L,
                                          double& new_theta, double& new_phi);
extern void CylinderVel2AngularVel_Ankle_NewSW(double theta, double phi, double dx_R, double dx_L,
                                               double& dtheta, double& dphi);
extern void CylinderForce2Torque_Ankle_NewSW(double theta, double phi, double F_R, double F_L,
                                             double& T_pitch, double& T_roll);

extern void Rotary2Joint_QuadKnee(double S, double dS, double F,
                                  double &theta, double &dtheta, double &T);

InEKF_Parameter  InEKF_IMU;

int argInt[4] = {0,}; // Argument for SendGeneralMSG Function
double argDouble[4] = {0,}; // Argument for SendGeneralMSG Function

void CatchSignals(int _signal)
{

    switch(_signal)
    {
    case SIGHUP:     // shell termination
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
//        canHandler->Finish();
        usleep(1000*1000);

        for(int i=1; i<_NO_OF_AL; i++){
            pmHandler->CloseAL(i);
            //            cout<<"_NO_OF_AL :" <<_NO_OF_AL << endl;
        }
        IS_WORKING = false;
        //        std::cout << "daemon CatchSignals " << std::endl;


        break;
    }
    usleep(1000*1000);
}

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int extrafinger = 0;
    while((opt = getopt(argc, argv, "g:f:r:e:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                //                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                //                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'f':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
//                __IS_FOG = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
//                __IS_FOG = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for FOG";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'r':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                //                __IS_ROS = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                //                __IS_ROS = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for ROS";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'f'){
                FILE_LOG(logERROR) << "Option for FOG";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }
        }
    }

    cout << endl;
    //    FILE_LOG(logERROR) << "=========Daemon Setting==========";
    //    FILE_LOG(logWARNING) << argv[0];
    //    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "Daemon for Gazebo";
    //    else                FILE_LOG(logWARNING) << "Daemon for Robot";
    //    if(__IS_FOG)        FILE_LOG(logWARNING) << "FOG is used";
    //    else                FILE_LOG(logWARNING) << "FOG is not used";
    //    if(__IS_ROS)        FILE_LOG(logWARNING) << "ROS is used";
    //    else                FILE_LOG(logWARNING) << "ROS is not used";
    //    FILE_LOG(logERROR) << "=================================";
    //    cout << endl;
}

int main(int argc, char *argv[])
{

    // Copyright
    cout << endl;
    cout << " \033[31m######################################################################\n";
    cout << " #                                                                    #\n";
    cout << " #  PODO Version 2.2                                                  #\n";
    cout << " #  Copyright 2016 Rainbow Robotics Co.                               #\n";
    cout << " #                                                                    #\n";
    cout << " #  Main developer: Jeongsoo Lim                                      #\n";
    cout << " #  E-mail: yjs0497@kaist.ac.kr                                       #\n";
    cout << " #                                                                    #\n";
    cout << " #  We touch the core!                                                #\n";
    cout << " #                                                                    #\n";
    cout << " ######################################################################\n";

    // Termination signal
    signal(SIGTERM, CatchSignals);       // "kill" from shell
    signal(SIGINT,  CatchSignals);       // Ctrl-c
    signal(SIGHUP,  CatchSignals);       // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping
    mlockall(MCL_CURRENT|MCL_FUTURE);

    CheckArguments(argc, argv);

    if(RBCore_Initialize() == false){
        FILE_LOG(logERROR) << "Core Initialization Failed..";
        return 0;
    }

    // Parameters Initialization =========================================================
    dec(cout); // decimal number print out
    // MPC parameters setting (Pump Control)
    sharedREF->N_PrevPump = 26;
    sharedREF->dT_PrevPump = 0.060;
    sharedREF->Flag_PumpControlMPC = false;
    for(int i=0;i<=MAX_PREVIEW;i++) {
        for(int j=0;j<MAX_VC;j++) {
            sharedREF->LoadPressureReference_Future[i][j] = 35.0;
            sharedREF->ActFlowrateReference_Future[i][j] = 0.0;
        }
    }
    sharedSEN->PUMP[0].CurrentSettingVoltage = 100.0;
    // ===================================================================================

    unsigned int whileindex = 0;
    while(IS_WORKING){
        usleep(100*1000); // 100ms

        switch(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND){
        case DAEMON4LIGHT_PROCESS_CREATE:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PROCESS_CREATE";
            pmHandler->OpenAL(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_PROCESS_KILL:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PROCESS_KILL";
            pmHandler->CloseAL(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_CAN_CHECK:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_CAN_CHECK";
            if(IS_CAN_READY)   {RBCMD_CAN_Check();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
//        case DAEMON4LIGHT_CAN_CHANNEL_ARRANGE:
//        {
//            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_CAN_CHANNEL_ARRANGE";
//            if(IS_CAN_READY)   {RBCMD_CAN_Channel_Arrange();}
//            else            {FILE_LOG(logWARNING) << "CAN device not set";}
//            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
//            break;
//        }

        case DAEMON4LIGHT_SAVEDATA:
        {
            if(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]) {
                save_Flag = true;
                FILE_LOG(logWARNING) << "SAVING DAEMON DATA START!!";
            } else {
                save_Flag = false;
                save_File();
                FILE_LOG(logERROR) << "SAVING DAEMON DATA END!!";
            }

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_ERRORCLEAR:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_ERRORCLEAR";
            if(IS_CAN_READY)   {
                for(int i=0; i<_NO_OF_VC; i++)  {
                    _DEV_VC[i].CMD_ErrorClear();
                }
            }
            else {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_MOTION_ENCODER_ZERO:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_ENCODER_ZERO";
            if(IS_CAN_READY)   {RBCMD_EncoderZero();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_MOTION_FET_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_FET_ONOFF";
            if(IS_CAN_READY)   {RBCMD_InitFETOnOff();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_MOTION_REF_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_REF_ONOFF";
            if(IS_CAN_READY)   {RBCMD_MotionRefOnOff();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_BOARDTEST:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_BOARDTEST";
            _DEV_VC[0].InformationCheck();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING";
            if(IS_CAN_READY)   {
                int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
                if(_BNO == -1) {
                    FILE_LOG(logSUCCESS) << "All Torque(Force) Sensor Nulling!";
                    for(int i=0; i<_NO_OF_VC; i++)  {
                        _DEV_VC[i].CMD_ControlMode(20);
                    }
                } else {
                    FILE_LOG(logSUCCESS) << "Board(" << _BNO <<") Torque(Force) Sensor Nulling!";
                    _DEV_VC[_BNO].CMD_ControlMode(20);
                }
                SetWaitTime_us(5000000); // WaitDisplay ON for 5sec (5000000us)
            }
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_REF_RESET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_REF_RESET";
            if(IS_CAN_READY)   {
                FILE_LOG(logWARNING) << "All Position & Torque Reference Reset!";
                bool SupplyPressureChangeOnOff = sharedREF->PumpSupplyPressureChange;
                for(int i=0; i<_NO_OF_VC; i++)  {
                    if(_DEV_VC[i].ResetReference_PosVel(SupplyPressureChangeOnOff))
                    {
                        FILE_LOG(logINFO) << "Borad(" << i << ") Reference Reset.";
                    }
                }
            }
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_REQUEST_ONOFF:
        {
            //            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_REQUEST_ONOFF";
            RBCMD_ValveControllerRequestOnOff();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_MOTION_ASK_PARAMETERS:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_ASK_PARAMETERS";
            int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            RBCMD_AskValveControllerParameters(_BN);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_SET_PARAMETERS:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGT_MOTION_SET_EVERYTHING";
            int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            HCB_INFO H = sharedCMD->COMMAND[RBCORE_PODO_NO].HCB_Info;
            RBCMD_SetValveControllerParameters(_BN, H);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_READnSAVE_PARAMETERS:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGT_MOTION_SET_EVERYTHING";
            RBCMD_ReadnSaveValveControllerParameters();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_LOADnSET_PARAMETERS:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGT_MOTION_SET_EVERYTHING";
            RBCMD_LoadnSetValveControllerParameters();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_SET_BNO:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGT_MOTION_SET_BNO";
            int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            int target_BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
            argInt[0] = target_BNO;
            _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_BOARDNUMBER, argInt);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_CHANGE_POSorTOR:
        {
            // Position Ctrl or Force Ctrl
            for(int i=0;i<12;i++) {
                sharedREF->PosOrFor_Selection[i] = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[i];
                if(sharedREF->PosOrFor_Selection[i]!=sharedREF->PosOrFor_Selection_last[i]) FILE_LOG(logINFO) << "Joint("<< i <<") Control Mode Change!";
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_CHANGE_SUPPLYPRES_MODE:
        {
            // Constant Supply Pressure (0) or Variable Supply Pressure (1)
            sharedREF->PumpSupplyPressureChange = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            for(int i=0;i<_NO_OF_VC;i++) {
                argInt[0] = sharedREF->PumpSupplyPressureChange;
                _DEV_VC[i].SendGeneralMSG(ValveController_GeneralMSG_CMD_VARIABLESUPPLY_ONOFF,argInt);
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        // Valve Performance Test Command //////////////////////////////////////////////////

        case DAEMON4LIGHT_VALVEPERFTEST_REQUEST_ONOFF:
        {
            int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            bool enable = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
            int type = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];

            if(enable) cout << "Request to Get Data from Board(" << _BNO <<")"<< endl;
            else cout << "Stop Getting Data from Board(" << _BNO <<")"<< endl;
            switch (type)
            {
            case 0:
                cout << "Data Type : Position & Velocity & Force (Torque)" << endl;
                _DEV_VC[_BNO].CMD_Request_PosVel(enable);
                break;
            case 1:
                cout << "Data Type : Valve Position & PWM" << endl;
                _DEV_VC[_BNO].CMD_Request_ValvePosnPWM(enable);
                break;
            case 2:
                cout << "Data Type : Actuator Pressure" << endl;
                _DEV_VC[_BNO].CMD_Request_Pressure(enable);
                break;
            case 3:
                cout << "Data Type : Something for Debugging" << endl;
                _DEV_VC[_BNO].CMD_Request_OtherInfo(enable);
                break;
            case -1:
                cout << "Data Type : ALL DATA" << endl;
                _DEV_VC[_BNO].CMD_Request_PosVel(enable);
                usleep(10*1000);
                _DEV_VC[_BNO].CMD_Request_ValvePosnPWM(enable);
                usleep(10*1000);
                _DEV_VC[_BNO].CMD_Request_Pressure(enable);
                usleep(10*1000);
                _DEV_VC[_BNO].CMD_Request_OtherInfo(enable);
                usleep(10*1000);
                break;
            default:
                break;
            }

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_VALVEPERFTEST_VOLTAGE2VALVEPOS_ID:
        {
            int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
//            if(_DEV_VC[_BNO].VALVE_TYPE == "DDV") {
//                _DEV_VC[_BNO].CMD_ControlMode(30);
//                FILE_LOG(logSUCCESS) << "Input Voltage VS Valve Pos. ID is started!";
//            } else {
//                FILE_LOG(logERROR) << "Input Voltage VS Valve Pos. ID is not allowed.";
//                FILE_LOG(logERROR) << "Valve type of this board : " << _DEV_VC[_BNO].VALVE_TYPE.toUtf8().constData();
//            }
            _DEV_VC[_BNO].CMD_ControlMode(30);
            FILE_LOG(logSUCCESS) << "Input Voltage VS Valve Pos. ID is started!";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_VALVEPERFTEST_VOLTAGE2VALVEPOS_ASKRESULT:
        {
            int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            argInt[0] = whileindex;
            _DEV_VC[_BNO].SendGeneralMSG(ValveController_GeneralMSG_ASK_VOLTAGE2VALVEPOS_RESULT,argInt);
            whileindex++;
            if(whileindex >= 41)
            {
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
                whileindex = 0;
            }
//            if(_DEV_VC[_BNO].VALVE_TYPE == "DDV") {
//                argInt[0] = whileindex;
//                _DEV_VC[_BNO].SendGeneralMSG(ValveController_GeneralMSG_ASK_VOLTAGE2VALVEPOS_RESULT,argInt);
//                whileindex++;
//                if(whileindex >= 31)
//                {
//                    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
//                    whileindex = 0;
//                }
//            } else {
//                FILE_LOG(logERROR) << "Input Voltage VS Valve Pos. ID is not allowed.";
//                FILE_LOG(logERROR) << "Valve type of this board : " << _DEV_VC[_BNO].VALVE_TYPE.toUtf8().constData();
//            }
            break;
        }

        case DAEMON4LIGHT_VALVEPERFTEST_DEADZONE_ID:
        {
            int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            _DEV_VC[_BNO].CMD_ControlMode(31);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_VALVEPERFTEST_DEADZONE_ASKRESULT:
        {
            int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            _DEV_VC[_BNO].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEDZ);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_VALVEPERFTEST_VALVEPOS2FLOWRATE_ID:
        {
            int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            _DEV_VC[_BNO].CMD_ControlMode(32);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_VALVEPERFTEST_VALVEPOS2FLOWRATE_ASKRESULT:
        {
            int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            argInt[0] = whileindex;
            _DEV_VC[_BNO].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEPOS2FLOWRATE_RESULT, argInt);
            whileindex++;
            if(whileindex >= 51)
            {
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
                whileindex = 0;
            }
            break;
        }

        // Pump Command //////////////////////////////////////////////////

        case DAEMON4LIGHT_PUMP_ASK_STATUS:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PUMP_ASK_STATUS";
            int CMD_TYPE = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

            if(CMD_TYPE == PumpController_GeneralMSG_ASK_SPEEDREF)
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_ASK_SPEEDREF);
            if(CMD_TYPE == PumpController_GeneralMSG_ASK_CTRL_ONOFF)
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_ASK_CTRL_ONOFF);
            if(CMD_TYPE == PumpController_GeneralMSG_ASK_DATAREQUESTFLAG)
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_ASK_DATAREQUESTFLAG);

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_PUMP_SEND_COMMAND:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PUMP_SEND_COMMAND";
            int CMD_TYPE = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];
            int CMD_DATA = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[1];

            if(CMD_TYPE == PumpController_GeneralMSG_CMD_CTRL_ON) {
                sharedSEN->PUMP[0].CurrentSettingVoltage = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_DOUBLE[0];
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_CMD_CTRL_ON);
                sharedREF->PumpCtrlMode_Command[0] = PumpControlMode_Interpolation;
            }
            if(CMD_TYPE == PumpController_GeneralMSG_CMD_CTRL_OFF) {
                argInt[0] = 0; // Pump Speed Command
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_CMD_SPEEDREF,argInt);
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_CMD_CTRL_OFF);
            }
            if(CMD_TYPE == PumpController_GeneralMSG_CMD_PRESSURENULL) {
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_CMD_PRESSURENULL);
            }
            if(CMD_TYPE == PumpController_GeneralMSG_CMD_DATAREQUESTFLAG) {
                argInt[0] = CMD_DATA; // Request On/Off
                _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_CMD_DATAREQUESTFLAG, argInt);
            }

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET";
            int REF_SPEED = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];
            int REF_SPEED_MIN = 150;
            int REF_SPEED_MAX = (int)(0.95*sharedSEN->PUMP[0].CurrentSettingVoltage/100.0*3000.0);
            if(REF_SPEED > REF_SPEED_MAX) {
                REF_SPEED = REF_SPEED_MAX;
                FILE_LOG(logWARNING) << "Pump speed reference is saturated! (Maximum : " << REF_SPEED_MAX << " rpm)";
            } else if (REF_SPEED < REF_SPEED_MIN) {
                REF_SPEED = REF_SPEED_MIN;
                FILE_LOG(logWARNING) << "Pump speed reference is saturated! (Minimum : " << REF_SPEED_MIN << " rpm)";
            }
            FILE_LOG(logINFO) << "Ref Pump Speed : " << REF_SPEED;

            argInt[0] = REF_SPEED; // Pump Speed Command
            _DEV_PC[0].SendGeneralMSG(PumpController_GeneralMSG_CMD_SPEEDREF,argInt);
            sharedREF->PumpVelocityReference[0] = (double)REF_SPEED;

            sharedREF->PumpCtrlMode_Command[0] = PumpControlMode_Interpolation;

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        // IMU Sensor Setting //////////////////////////////////////////////////

        case DAEMON4LIGHT_SENSOR_IMU_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_IMU_ONOFF";
            if(IS_CAN_READY)   {
                char ONOFF = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
                _DEV_IMU[0].RBIMU_RequestONOFF(ONOFF);
            }
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_SENSOR_IMU_NULL:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_IMU_ONOFF";
            if(IS_CAN_READY)   {
                _DEV_IMU[0].RBIMU_Nulling();
            }
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET";
            if(IS_CAN_READY)   {
                double t = 0.0;
                int cnt = 0;
                double roll_sum = 0.0;
                double pitch_sum = 0.0;

                while(t<5.0) {
                    cnt++;
                    roll_sum += sharedSEN->IMU[0].Roll;
                    pitch_sum += sharedSEN->IMU[0].Pitch;
                    usleep(1000); // 1msec
                    t += 0.001;
                    if(cnt%1000 == 0) { FILE_LOG(logWARNING) << "IMU Offset Setting... Remaining " << (5-cnt/1000) << "sec.";}
                }
                _DEV_IMU[0].ROLL_OFFSET = roll_sum/(double)cnt;
                _DEV_IMU[0].PITCH_OFFSET = pitch_sum/(double)cnt;

                FILE_LOG(logSUCCESS) << "IMU offset is found! ";
                FILE_LOG(logSUCCESS) << "Roll offset : " << _DEV_IMU[0].ROLL_OFFSET*R2D << "(deg)";
                FILE_LOG(logSUCCESS) << "Pitch offset : " << _DEV_IMU[0].PITCH_OFFSET*R2D << "(deg)";
            }
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        // FT Sensor Setting //////////////////////////////////////////////////

        case DAEMON4LIGHT_SENSOR_FT_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_FT_ONOFF";
            if(IS_CAN_READY)   {
                char ONOFF = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
                for(int i=0; i<_NO_OF_FT; i++){
                    _DEV_FT[i].RBFT_RequestONOFF(ONOFF);
                }
            }
            else {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_SENSOR_FT_NULL:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_FT_ONOFF";
            if(IS_CAN_READY)   {
                for(int i=0; i<_NO_OF_FT; i++){
                    _DEV_FT[i].RBFT_Nulling(0);
                }
            }
            else {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        // ///////////////////////////////////////////////////////////////////

        case DAEMON4LIGHT_ENC_ZERO:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_ENC_ZERO";
            RBCMD_EncZero();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_FINDHOME:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_HOME_POS";
            RBCMD_FindHome();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_SIMLATION_MODE_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SIMLATION_MODE_ONOFF";
            // __IS_CHOREONOID == 0 : Robot mode
            // __IS_CHOREONOID == 1 : Simulation(choreonoid) mode
            sharedREF->Simulation_DataEnable = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];

            if (sharedREF->Simulation_DataEnable == 0) FILE_LOG(logINFO) << "MODE : ROBOT";
            else if (sharedREF->Simulation_DataEnable == 1)  FILE_LOG(logINFO) << "MODE : CHOREONOID";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        }

        // Command acceptance check for AL -------------------------------
        //        for(int i=2; i<_NO_OF_AL; i++){ // i=0 Daemon, i=1 PODOLAN
        //            if(sharedCMD->COMMAND[i].USER_COMMAND != 0 &&
        //                sharedCMD->COMMAND[i].USER_COMMAND != 100 &&
        //                sharedCMD->CommandAccept[i] == false){ // NO_ACT should be 0 or 100
        //                    _ALCommandCnt[i]++;
        //            }else{
        //                _ALCommandCnt[i] = 0;
        //            }

        //            // AL didn't accept command for (100ms X 5 = 500ms)
        //            if(_ALCommandCnt[i] > 5){
        //                sharedCMD->ErrorInform2GUI |= (1<<i);
        //                sharedCMD->CommandAccept[i] = true;
        //            }
        //        }
        // ----------------------------------------------------------------
    }


    RBCore_Termination();

    usleep(1000*1000);
    return 0;
}



void *RBCore_RTThreadCon(void *)
{
    timespec TIME_READ;
    timespec TIME_FLAG;
    timespec TIME_WRITE;
    timespec TIME_SHOW;

    timespec TIME_NOW;
    timespec TIME_NEXT;

    const long PERIOD_US = RT_TIMER_PERIOD_US;
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while(IS_WORKING)
    {
        static int Daemon_ThreadCycle = 0;
        static timespec TIME_TIC;
        static timespec TIME_TOC;
        static int CNT_TICTOC = 0;
        CNT_TICTOC++;
        if(CNT_TICTOC%2==0) {
            clock_gettime(CLOCK_REALTIME, &TIME_TIC);
            Daemon_ThreadCycle = timediff_us(&TIME_TOC, &TIME_TIC);
//            if(Daemon_ThreadCycle > RT_TIMER_PERIOD_US)
//                cout << "Cycle Time (Daemon Thread) : " << (double)Daemon_ThreadCycle*0.001 << " ms " << endl;
            CNT_TICTOC = 0;
        } else {
            clock_gettime(CLOCK_REALTIME, &TIME_TOC);
            Daemon_ThreadCycle = timediff_us(&TIME_TIC, &TIME_TOC);
//            if(Daemon_ThreadCycle > RT_TIMER_PERIOD_US)
//            cout << "Cycle Time (Daemon Thread) : " << (double)Daemon_ThreadCycle*0.001 << " ms " << endl;
        }

        // Read Sensor & Encoder =====================================
        clock_gettime(CLOCK_REALTIME, &TIME_READ);
        if(sharedREF->Simulation_DataEnable==0) { // robot mode (by BUYOUN)

            // Valve Controller Data Read
            ReadBoardInfo(); // Update HCB_Info
            ReadActuatorData();
            ReadValvePosnPWM();
            ReadValvePressure();
            ReadValveOtherData();
            ReadValveAlart();

            // Pump Controller Data Read
            ReadPumpData();

            // Sensors Data Read
            ReadNewIMU();
            ReadFT();

        } else if(sharedREF->Simulation_DataEnable==1) { // simulation mode (by BUYOUN)
            ReadSimulationData();
        }
        // ===========================================================

        // Change Flag ===============================================
        clock_gettime(CLOCK_REALTIME, &TIME_FLAG);

//        const int AL_Number_PUMPING = 2;
//        const int AL_Number_WALKING = 3;3
//        const int AL_Number_SH_TASK = 4;
        const int AL_Number_VALVETEST = 2;
//        static int CntFlagThread_PUMPING = 0;
//        static int CntFlagThread_WALKING = 0;
//        static int CntFlagThread_SH_TASK = 0;
        static int CntFlagThread_VALVETEST = 0;

//        // Pumping AL Flag
//        if(sharedCMD->ALTHREAD_ONOFF_SIGNAL[AL_Number_PUMPING]) {
//            CntFlagThread_PUMPING++;
//            if(CntFlagThread_PUMPING%2 == 0) { // Daemon : 400 Hz, PumpingAL : 200 Hz
//                CntFlagThread_PUMPING = 0;
//                sharedCMD->SYNC_SIGNAL[AL_Number_PUMPING] = true;
//            } else {

//            }
//        }

//        // Walking AL Flag
//        if(sharedCMD->ALTHREAD_ONOFF_SIGNAL[AL_Number_WALKING]) {
////            FILE_LOG(logDEBUG) << "Thread Test (Daemon)";
//            CntFlagThread_WALKING++;
//            if(CntFlagThread_WALKING%2 == 0) { // Daemon : 400 Hz, LIGHTWalkingAL : 200 Hz
//                CntFlagThread_WALKING = 0;
//                for(int i=0; i<_NO_OF_VC; i++){
//                    for(int j=0; j<1; j++){
//                        sharedCMD->ACK_SIGNAL[i][j] = false;
//                    }
//                }
//                sharedCMD->SYNC_SIGNAL[AL_Number_WALKING] = true;
//            } else {

//            }
//        }

//        // Actuator Test AL Flag
//        if(sharedCMD->ALTHREAD_ONOFF_SIGNAL[AL_Number_SH_TASK]) {
//            CntFlagThread_SH_TASK++;
//            if(CntFlagThread_SH_TASK%1 == 0) { // Daemon : 400 Hz, SH_TASK_AL : 400 Hz
//                CntFlagThread_SH_TASK = 0;
//                for(int i=0; i<_NO_OF_VC; i++){
//                    for(int j=0; j<1; j++){
//                        sharedCMD->ACK_SIGNAL[i][j] = false;
//                    }
//                }
//                sharedCMD->SYNC_SIGNAL[AL_Number_SH_TASK] = true;
//            } else {

//            }
//        }

        // Valve Performance Test AL Flag
        if(sharedCMD->ALTHREAD_ONOFF_SIGNAL[AL_Number_VALVETEST]) {
            CntFlagThread_VALVETEST++;
            if(CntFlagThread_VALVETEST%1 == 0) { // Daemon : 400 Hz, VALVETEST_AL : 400 Hz
                CntFlagThread_VALVETEST = 0;
                for(int i=0; i<_NO_OF_VC; i++){
                    for(int j=0; j<1; j++){
                        sharedCMD->ACK_SIGNAL[i][j] = false;
                    }
                }
                sharedCMD->SYNC_SIGNAL[AL_Number_VALVETEST] = true;
            } else {

            }
        }

//        for(int i=0; i<_NO_OF_VC; i++){
//            for(int j=0; j<1; j++){
//                sharedCMD->ACK_SIGNAL[i][j] = false;
//            }
//        }
//        for(int i=0; i<_NO_OF_AL; i++){
//            sharedCMD->SYNC_SIGNAL[i] = true;
//        }

        // ===========================================================

        // Wait Reference ============================================
        timespec th_start, th_stop;
        int     waitCnt, waitOK;
        double  timeGap = 0.0;

        clock_gettime(CLOCK_REALTIME, &th_start);
        waitCnt = 0;
        waitOK = false;
        while(1){
            // check the all WriteDoneFlag are enabled
            int notRead = 0;
            for(int i=0; i<_NO_OF_VC; i++){
                for(int j=0; j<1; j++){
                    if(sharedCMD->ACK_SIGNAL[i][j] == false && sharedCMD->MotionOwner[i][j] != RBCORE_PODO_NO){
                        notRead++;
                    }
                }
            }
            if(notRead == 0){
                clock_gettime(CLOCK_REALTIME, &th_stop);
                timeGap = (double)timediff_us(&th_start, &th_stop);
//                FILE_LOG(logINFO) << "Reference Delay : " << timeGap <<" [us]";
//                FILE_LOG(logINFO) << "[Daemon] RESPONSE_CNT_AL : " << sharedREF->RESPONSE_CNT_AL;
//                FILE_LOG(logINFO) << "waitCnt : " << waitCnt ;
//                FILE_LOG(logINFO) << "======================= : ";
                waitOK = true;
                break;
            } else {
                if(waitCnt > 500){
                    clock_gettime(CLOCK_REALTIME, &th_stop);
                    timeGap = (double)timediff_us(&th_start, &th_stop) / 1000.0;
                    if(timeGap > 2.5){
                        waitOK = false;
//                        FILE_LOG(logWARNING) << "Over 2.5msec";
                        break;
                    }
                }
                waitCnt++;
                usleep(2);
            }
        }
        // ===========================================================

        // Write CAN Reference =======================================
        clock_gettime(CLOCK_REALTIME, &TIME_WRITE);

        // Motor(Valve) Controller Reference
        WriteValveReference();

        // Pump Controller Reference
        WritePumpReference();

        // ===========================================================

        // Variable Display for debuging =======================================
        clock_gettime(CLOCK_REALTIME, &TIME_SHOW);
        _ThreadCnt++;
        if (_ThreadCnt%500 == 0) {
            //            for(int i=0;i<12;i++) {
            //                int16_t temp1 = _DEV_VC[0].Joints[0].HCB_Data.CurrentTempData1;
            //                int16_t temp2 = _DEV_VC[0].Joints[0].HCB_Data.CurrentTempData2;
            //                FILE_LOG(logDEBUG3) << "K : (" << i << ") : "<< temp1;
            //                FILE_LOG(logDEBUG3) << "D : (" << i << ") : "<< temp2;
            //            }

            //            FILE_LOG(logDEBUG3) << "Ps : " << sharedREF->PumpPressureReference[0];
            //            FILE_LOG(logDEBUG3) << "Ps : " << _DEV_VC[0].Joints[0].HCB_Ref.ReferencePumpPressure;
            //            FILE_LOG(logDEBUG3) << "Change? : " << sharedREF->PumpSupplyPressureChange;
            _ThreadCnt = 0;
        }

        if(save_Flag == true){
            save_PutData(save_Index);
            save_Index++;
            if(save_Index == (SAVENUM-1))
                save_Index=0;
        }


        if(WaitTime_us>0) WaitDisplay_us();

        // =================================================================
        // Timer Handling
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);

//        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
//            FILE_LOG(logDEBUG) << "t_READ : " << timediff_ns(&TIME_READ, &TIME_FLAG)/1000 <<" us";
//            FILE_LOG(logDEBUG) << "t_FLAG : " << timediff_ns(&TIME_FLAG, &TIME_WRITE)/1000 <<" us";
//            FILE_LOG(logDEBUG) << "t_WRIT : " << timediff_ns(&TIME_WRITE, &TIME_SHOW)/1000 <<" us";
//            FILE_LOG(logDEBUG) << "t_SHOW : " << timediff_ns(&TIME_SHOW, &TIME_NOW)/1000 <<" us";
//            FILE_LOG(logDEBUG) << "RT Deadline Miss (Daemon Thread) : " << timediff_us(&TIME_NEXT, &TIME_NOW)<<" us";
//        }
    }
//    FILE_LOG(logERROR) << "RTthread will be terminated..";
}

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = (double)RT_TIMER_PERIOD_US/1000000.0;

//    save_Buf[1][cur_Index] = Daemon_ThreadCycle;

}

void SetWaitTime_us(int T) {
    WaitTime_us = T;
    FILE_LOG(logWARNING) << "Wait for " << WaitTime_us/1000000 << "sec.";
}
void WaitDisplay_us(void){
    // just display waiting time (buyoun)
    if(WaitTime_us%1000000 == 0) {
    } WaitTime_us = WaitTime_us - RT_TIMER_PERIOD_US;
    if(WaitTime_us<=0){
        FILE_LOG(logSUCCESS) << "The process is done!!!";
    }
}

// =================================================================================================
// =================================================================================================
// =================================================================================================

//void *RBCore_NRTThreadCon(void *)
//{
//    DRC_GAZEBO_SENSOR   GazeboSensor;
//    DRC_GAZEBO_JOINT    GazeboJoint;
//    char                *GazeboJoint_and_Type;
//    DRC_GAZEBO_GO_CMD   GazeboGain;
//    char                *GazeboGain_and_Type;

//    GazeboJoint_and_Type = (char*)malloc(sizeof(DRC_GAZEBO_JOINT) + sizeof(int));
//    GazeboGain_and_Type = (char*)malloc(sizeof(DRC_GAZEBO_GO_CMD) + sizeof(int));

//    RTIME   th_start, th_stop;
//    int     waitCnt, waitOK;
//    double  timeGap = 0.0;
//    double  finalJointRef[MOTOR_2CH];
//    //    double  finalJointPWMRef[MOTOR_2CH];
//    //    double  finalJointCurrentRef[MOTOR_2CH];

//    while(IS_WORKING)
//    {
//        usleep(10);
//        if(lanHandler->ConnectionStatus){
//            if(lanHandler->RBLanReadData((char*)(&GazeboSensor), sizeof(GazeboSensor), 0x00) == LAN_NEW_DATA){

//                sharedSEN->Sim_Time_sec = GazeboSensor.Sim_Time.sec;
//                sharedSEN->Sim_Time_nsec = GazeboSensor.Sim_Time.nsec;

//                // Read Sensor & Encoder =====================================
//                for(int i=0; i<_NO_OF_FT; i++){
//                    sharedSEN->FT[i].Fx = GazeboSensor.FTSensor[i].force[0];
//                    sharedSEN->FT[i].Fy = GazeboSensor.FTSensor[i].force[1];
//                    sharedSEN->FT[i].Fz = GazeboSensor.FTSensor[i].force[2];
//                    sharedSEN->FT[i].Mx = GazeboSensor.FTSensor[i].torque[0];
//                    sharedSEN->FT[i].My = GazeboSensor.FTSensor[i].torque[1];
//                    sharedSEN->FT[i].Mz = GazeboSensor.FTSensor[i].torque[2];
//                }
//                for(int i=0; i<_NO_OF_IMU; i++){
//                    sharedSEN->IMU[i].Roll      = GazeboSensor.IMUSensor[0];
//                    sharedSEN->IMU[i].Pitch     = GazeboSensor.IMUSensor[1];
//                    sharedSEN->IMU[i].Yaw       = GazeboSensor.IMUSensor[2];
//                    sharedSEN->IMU[i].Wx_B      = GazeboSensor.IMUSensor[3];
//                    sharedSEN->IMU[i].Wy_B      = GazeboSensor.IMUSensor[4];
//                    sharedSEN->IMU[i].Wz_B      = GazeboSensor.IMUSensor[5];
//                    sharedSEN->IMU[i].Ax_B      = GazeboSensor.IMUSensor[6];
//                    sharedSEN->IMU[i].Ay_B      = GazeboSensor.IMUSensor[7];
//                    sharedSEN->IMU[i].Az_B      = GazeboSensor.IMUSensor[8];
//                }

//                double tempDouble;
//                for(int i=0; i<NO_OF_JOINTS; i++){
//                    tempDouble = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngle;
//                    sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngle = GazeboSensor.JointCurrentPosition[i];
//                    sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngVel = (sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngle - tempDouble) / (double)RT_TIMER_PERIOD_MS * 1000.0;
//                }


//                sharedSEN->FOG.Roll     = sharedSEN->IMU[0].Roll;
//                sharedSEN->FOG.Pitch    = sharedSEN->IMU[0].Pitch;
//                sharedSEN->FOG.Yaw      = sharedSEN->IMU[0].Yaw;
//                sharedSEN->FOG.RollVel     = sharedSEN->IMU[0].Wx_B*RBCORE_PI/180.;
//                sharedSEN->FOG.PitchVel    = sharedSEN->IMU[0].Wy_B*RBCORE_PI/180.;
//                sharedSEN->FOG.YawVel      = sharedSEN->IMU[0].Wz_B*RBCORE_PI/180.0;

//                sharedSEN->CAN_Enabled = _CANOUT_ENABLED;
//                sharedSEN->SEN_Enabled = _SENSOR_ENABLED;
//                // ===========================================================

//                // Change Flag ===============================================
//                for(int i=0; i<_NO_OF_VC; i++){
//                    for(int j=0; j<1; j++){
//                        sharedCMD->ACK_SIGNAL[i][j] = false;
//                    }
//                }
//                for(int i=0; i<_NO_OF_AL; i++){
//                    sharedCMD->SYNC_SIGNAL[i] = true;
//                }
//                // ===========================================================

//                // Wait Reference ============================================
//                th_start = rt_timer_read();
//                waitCnt = 0;
//                waitOK = false;
//                while(1){
//                    // check the all WriteDoneFlag are enabled
//                    int notRead = 0;
//                    for(int i=0; i<_NO_OF_VC; i++){
//                        for(int j=0; j<1; j++){
//                            if(sharedCMD->ACK_SIGNAL[i][j] == false && sharedCMD->MotionOwner[i][j] != RBCORE_PODO_NO){
//                                notRead++;
//                            }
//                        }
//                    }
//                    if(notRead == 0){
//                        th_stop = rt_timer_read();
//                        timeGap = (double)(th_stop - th_start) / (double)1000000.0;
//                        waitOK = true;
//                        break;
//                    }

//                    if(waitCnt%500 == 0){
//                        th_stop = rt_timer_read();
//                        timeGap = (double)(th_stop - th_start) / (double)1000000.0;
//                        if(timeGap > 2.5){
//                            waitOK = false;
//                            FILE_LOG(logWARNING) << "Over 2.5msec";
//                            break;
//                        }
//                    }
//                    waitCnt++;
//                    usleep(2);
//                }
//                // ===========================================================

//                // Write LAN Reference =======================================
//                for(int i=0; i<_NO_OF_VC; i++){
//                    for(int j=0; j<1; j++){
//                        int motionOwner = sharedCMD->MotionOwner[i][j];

//                        finalJointRef[j] = sharedREF->AngleReference[motionOwner][i][j];
//                        _DEV_VC[i].Joints[j].HCB_Ref.ReferencePosition = finalJointRef[j];
//                        sharedSEN->ENCODER[i][j].CurrentRefAngle = finalJointRef[j];

//                        //                        finalJointPWMRef[j] = sharedREF->JointFFpwm[motionOwner][i][j];
//                        //                        _DEV_VC[i].Joints[j].RefPWM = finalJointPWMRef[j];
//                        //                        _DEV_VC[i].Joints[j].RefFFpwm = finalJointPWMRef[j];
//                        //                        sharedSEN->ENCODER[i][j].PWMffout = finalJointPWMRef[j];

//                        //                        finalJointCurrentRef[j] = sharedREF->COCOAQCurrent_REF[motionOwner][i];
//                        //                        _DEV_VC[i].Joints[j].RefQCurrent = finalJointCurrentRef[j];
//                        //                        sharedSEN->ENCODER[i][j].CurrentQCurrentReference = finalJointCurrentRef[j];
//                    }
//                }
//                for(int i=0; i<NO_OF_JOINTS; i++){
//                    GazeboJoint.JointReference[i] = _DEV_VC[MC_GetID(i)].Joints[MC_GetCH(i)].HCB_Ref.ReferencePosition;
//                }
//                int type = GAZEBO_TYPE_JOINT;
//                //memcpy(GazeboJoint_and_Type, &type, sizeof(int));
//                //memcpy(&(GazeboJoint_and_Type[sizeof(int)]), &GazeboJoint, sizeof(DRC_GAZEBO_JOINT));

//                lanHandler->RBLanWriteData(&type, sizeof(int));
//                lanHandler->RBLanWriteData(&GazeboJoint, sizeof(DRC_GAZEBO_JOINT));
//                //lanHandler->RBLanWriteData(GazeboJoint_and_Type, sizeof(GazeboJoint_and_Type));
//                // ===========================================================

//                for(int i=0; i<MAX_MANUAL_CAN; i++){
//                    if(sharedCMD->ManualCAN[i].status == MANUALCAN_NEW){
//                        sharedCMD->ManualCAN[i].status = MANUALCAN_WRITING;

//                        int id = sharedCMD->ManualCAN[i].id;
//                        int dlc = sharedCMD->ManualCAN[i].dlc;
//                        int bno = -1;
//                        for(int j=0; j<_NO_OF_VC; j++){
//                            if(_DEV_VC[j].ID_SEND_GENERAL == id){
//                                bno = j;
//                                break;
//                            }
//                        }

//                        if(bno >= 0){
//                            if(dlc > 0){
//                                if(sharedCMD->ManualCAN[i].data[0] == 0x6F){    // Gain Override
//                                    int ch = sharedCMD->ManualCAN[i].data[1]-1;
//                                    int joint = -1;
//                                    for(int j=0; j<NO_OF_JOINTS; j++){
//                                        if(MC_GetID(j) == bno && MC_GetCH(j) == ch){
//                                            joint = j;
//                                            break;
//                                        }
//                                    }
//                                    //FILE_LOG(logWARNING) << "ManualCAN JOINT: " << joint;
//                                    if(joint >= 0){
//                                        int gain = sharedCMD->ManualCAN[i].data[2];
//                                        int timeMS = (int)(sharedCMD->ManualCAN[i].data[3] | (sharedCMD->ManualCAN[i].data[4] << 8));

//                                        int type = GAZEBO_TYPE_GAINOVERRIDE;
//                                        GazeboGain.gain = gain;
//                                        GazeboGain.joint = joint;
//                                        GazeboGain.timeMs = timeMS;
//                                        FILE_LOG(logWARNING) << "GainOverride: " << joint << " , " << gain << ", " << timeMS;
//                                        lanHandler->RBLanWriteData(&type, sizeof(int));
//                                        lanHandler->RBLanWriteData(&GazeboGain, sizeof(DRC_GAZEBO_GO_CMD));
//                                    }
//                                }else if(sharedCMD->ManualCAN[i].data[0] == 0x11){ // Home
//                                    int ch = sharedCMD->ManualCAN[i].data[1]-1;
//                                    int joint = -1;
//                                    if(ch == -1){
//                                        FILE_LOG(logWARNING) << "Only support single channel";
//                                    }else{
//                                        for(int j=0; j<NO_OF_JOINTS; j++){
//                                            if(MC_GetID(j) == bno && MC_GetCH(j) == ch){
//                                                joint = j;
//                                                break;
//                                            }
//                                        }
//                                        if(joint >= 0){
//                                            int type = GAZEBO_TYPE_HOME;
//                                            FILE_LOG(logWARNING) << "Find Home: " << joint;
//                                            lanHandler->RBLanWriteData(&type, sizeof(int));
//                                            lanHandler->RBLanWriteData(&joint, sizeof(int));
//                                        }
//                                    }
//                                }
//                            }

//                        }
//                        sharedCMD->ManualCAN[i].status = MANUALCAN_EMPTY;
//                    }
//                }
//            }
//        }else{ // without Gazebo connection
//            // No Sensor Data

//            // Change Flag ===============================================
//            for(int i=0; i<_NO_OF_VC; i++){
//                for(int j=0; j<1; j++){
//                    sharedCMD->ACK_SIGNAL[i][j] = false;
//                }
//            }
//            for(int i=0; i<_NO_OF_AL; i++){
//                sharedCMD->SYNC_SIGNAL[i] = true;
//            }
//            // ===========================================================

//            // Wait Reference ============================================
//            th_start = rt_timer_read();
//            waitCnt = 0;
//            waitOK = false;
//            while(1){
//                // check the all WriteDoneFlag are enabled
//                int notRead = 0;
//                for(int i=0; i<_NO_OF_VC; i++){
//                    for(int j=0; j<1; j++){
//                        if(sharedCMD->ACK_SIGNAL[i][j] == false && sharedCMD->MotionOwner[i][j] != RBCORE_PODO_NO){
//                            notRead++;
//                        }
//                    }
//                }
//                if(notRead == 0){
//                    th_stop = rt_timer_read();
//                    timeGap = (double)(th_stop - th_start) / (double)1000000.0;
//                    waitOK = true;
//                    break;
//                }

//                if(waitCnt%500 == 0){
//                    th_stop = rt_timer_read();
//                    timeGap = (double)(th_stop - th_start) / (double)1000000.0;
//                    if(timeGap > 2.5){
//                        waitOK = false;
//                        FILE_LOG(logWARNING) << "Over 2.5msec";
//                        break;
//                    }
//                }
//                waitCnt++;
//                usleep(2);
//            }
//            // ===========================================================

//            // Move Reference ============================================
//            for(int i=0; i<_NO_OF_VC; i++){
//                for(int j=0; j<1; j++){
//                    int motionOwner = sharedCMD->MotionOwner[i][j];

//                    finalJointRef[j] = sharedREF->AngleReference[motionOwner][i][j];
//                    _DEV_VC[i].Joints[j].HCB_Ref.ReferencePosition = finalJointRef[j];
//                    sharedSEN->ENCODER[i][j].CurrentRefAngle = finalJointRef[j];
//                }
//            }
//            // ===========================================================

//            // No Manual Control

//            usleep(5*1000);
//        }
//    }
//    return NULL;
//}

// =================================================================================================
// =================================================================================================
// =================================================================================================

void save_File()
{
    FILE* fp;
    unsigned int i, j;
    fp = fopen("DataLog/LIGHT_Daemon_Data.txt", "w");

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

// =================================================================================================
// =================================================================================================
// =================================================================================================

void RBCMD_CAN_Check(){
    for(int i=0; i<_NO_OF_VC; i++){
        _DEV_VC[i].CANCheck();
        for(int j=0; j<MOTOR_2CH; j++){
            sharedSEN->ENCODER[i][j].BoardConnection = _DEV_VC[i].ConnectionStatus;
        }
    }
    for(int i=0; i<_NO_OF_PC; i++){
        _DEV_PC[i].CANCheck();
        sharedSEN->PUMP[i].BoardConnection = _DEV_PC[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_IMU; i++){
        _DEV_IMU[i].CANCheck();
        sharedSEN->IMU[i].BoardConnection = _DEV_IMU[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].CANCheck();
        sharedSEN->FT[i].BoardConnection = _DEV_FT[i].ConnectionStatus;
    }
}

void RBCMD_CAN_Channel_Arrange(){

//    // Motion Controller CAN Channel Arrangement
//    for(int i=0; i<_NO_OF_VC; i++){
//        _DEV_VC[i].CANChannel_Arrange();
//        usleep(100*1000);
//        for(int j=0; j<MOTOR_2CH; j++){
//            sharedSEN->ENCODER[i][j].BoardConnection = _DEV_VC[i].ConnectionStatus;
//        }
    }

//    // Pump Controller CAN Channel Arrangement
//    for(int i=0; i<_NO_OF_PC; i++){
//        _DEV_PC[i].CANChannel_Arrange();
//        usleep(100*1000);
//        sharedSEN->PUMP[i].BoardConnection = _DEV_PC[i].ConnectionStatus;
//    }

//    // IMU Sensor CAN Channel Arrangement
//    for(int i=0; i<_NO_OF_IMU; i++){
//        _DEV_IMU[i].CANChannel_Arrange();
//        usleep(100*1000);
//        sharedSEN->IMU[i].BoardConnection = _DEV_IMU[i].ConnectionStatus;
//    }

//    // FT Sensor CAN Channel Arrangement
//    for(int i=0; i<_NO_OF_FT; i++){
//        _DEV_FT[i].CANChannel_Arrange();
//        usleep(100*1000);
//        sharedSEN->FT[i].BoardConnection = _DEV_FT[i].ConnectionStatus;
//    }

//    RBDataBase DB;
//    DB.SetFilename("Core_Config.db");

//    std::cout << " \n\n========================================================\n";
//    std::cout << "           [CAN Channel Arrangement Result]        \n";
//    // Motion Controller DB Update
//    for(int i=0; i<_NO_OF_VC; i++){
//        int BOARD_ID = _DEV_VC[i].BOARD_ID;
//        int CAN_CH = _DEV_VC[i].CAN_CHANNEL;

//        if (_DEV_VC[i].ConnectionStatus) {
//            if(DB.UpdateDB_CAN_Channel_VC(BOARD_ID, CAN_CH) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << BOARD_ID << ": " << _DEV_VC[i].BOARD_NAME.toStdString().data() << ") is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
//        } else {
//            if(DB.UpdateDB_CAN_Channel_VC(BOARD_ID, 0) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << _DEV_VC[i].BOARD_ID << ": " << _DEV_VC[i].BOARD_NAME.toStdString().data() << ") is \033[31mnot connected.\033[0m \n";
//        }
//    }
//    // Pump Controller DB Update
//    for(int i=0; i<_NO_OF_PC; i++){
//        int BOARD_ID = _DEV_PC[i].BOARD_ID;
//        int CAN_CH = _DEV_PC[i].CAN_CHANNEL;

//        if (_DEV_PC[i].ConnectionStatus) {
//            if(DB.UpdateDB_CAN_Channel_PC(BOARD_ID, CAN_CH) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << BOARD_ID << ": PUMP) is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
//        } else {
//            if(DB.UpdateDB_CAN_Channel_PC(BOARD_ID, 0) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << _DEV_PC[i].BOARD_ID << ": PUMP) is \033[31mnot connected.\033[0m \n";
//        }
//    }
//    // IMU Controller DB Update
//    for(int i=0; i<_NO_OF_IMU; i++){
//        int BOARD_ID = _DEV_IMU[i].BOARD_ID;
//        int CAN_CH = _DEV_IMU[i].CAN_CHANNEL;

//        if (_DEV_IMU[i].ConnectionStatus) {
//            if(DB.UpdateDB_CAN_Channel_IMU(BOARD_ID, CAN_CH) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << BOARD_ID << ": IMU) is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
//        } else {
//            if(DB.UpdateDB_CAN_Channel_IMU(BOARD_ID, 0) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << _DEV_IMU[i].BOARD_ID << ": IMU) is \033[31mnot connected.\033[0m \n";
//        }
//    }
//    // FT Controller DB Update
//    for(int i=0; i<_NO_OF_FT; i++){
//        int BOARD_ID = _DEV_FT[i].BOARD_ID;
//        int CAN_CH = _DEV_FT[i].CAN_CHANNEL;

//        if (_DEV_FT[i].ConnectionStatus) {
//            if(DB.UpdateDB_CAN_Channel_FT(BOARD_ID, CAN_CH) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << BOARD_ID << ": "<< _DEV_FT[i].BOARD_NAME.toStdString().data() << ") is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
//        } else {
//            if(DB.UpdateDB_CAN_Channel_FT(BOARD_ID, 0) == false){
//                std::cout << "DB update fail!! \n";
//            }
//            std::cout << ">>> Board(" << _DEV_FT[i].BOARD_ID << ": " << _DEV_FT[i].BOARD_NAME.toStdString().data() << ") is \033[31mnot connected.\033[0m \n";
//        }
//    }
//    std::cout << " ========================================================\n\n";
//}

void RBCMD_EncoderZero(){
    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_VC; i++){
            for(int j=0; j<MAX_JOINT; j++){
                _DEV_VC[i].SendGeneralMSG(ValveController_GeneralMSG_CMD_ENCZERO);
                sharedCMD->MotionOwner[i][j] = RBCORE_PODO_NO;
                for(int k=0; k<_NO_OF_AL; k++){
                    sharedREF->AngleReference[k][i][j] = 0.0;
                }
                _DEV_VC[i].Joints[j].HCB_Ref.ReferencePosition = 0.0;
                _DEV_VC[i].Joints[j].HCB_Ref.ReferenceVelocity = 0.0;
            }
        }
    }else{  // Each
        _DEV_VC[id].SendGeneralMSG(ValveController_GeneralMSG_CMD_ENCZERO);
        sharedCMD->MotionOwner[id][ch] = RBCORE_PODO_NO;
        for(int k=0; k<_NO_OF_AL; k++){
            sharedREF->AngleReference[k][id][ch] = 0.0;
        }
        _DEV_VC[id].Joints[ch].HCB_Ref.ReferencePosition = 0.0;
        _DEV_VC[id].Joints[ch].HCB_Ref.ReferenceVelocity = 0.0;
    }
}
void RBCMD_InitFETOnOff(){
    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int onoff = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->off
    if(onoff != 0) onoff = 1;

    if(id == -1){ // All
        if(onoff == 1) {
            FILE_LOG(logINFO) << "ALL FET WILL TURN ON!";
        } else {
            FILE_LOG(logINFO) << "ALL FET WILL TURN OFF...";
        }

        for(int i=0; i<_NO_OF_VC; i++){
            argInt[0] = onoff;
            _DEV_VC[i].SendGeneralMSG(ValveController_GeneralMSG_CMD_FETONOFF,argInt);
        }
    }else{  // Each
        argInt[0] = onoff;
        _DEV_VC[id].SendGeneralMSG(ValveController_GeneralMSG_CMD_FETONOFF,argInt);
    }
}

void RBCMD_MotionRefOnOff()
{
    int Target = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int type = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    if(Target == -1) { // to all joints
        for(int i=0; i<_NO_OF_VC; i++){
            sharedREF->ValveCtrlMode_Command[i] = type;
        }
    } else if (Target==RAP||Target==RAR) {
        sharedREF->ValveCtrlMode_Command[RAP] = type;
        sharedREF->ValveCtrlMode_Command[RAR] = type;
    } else if (Target==LAP||Target==LAR) {
        sharedREF->ValveCtrlMode_Command[LAP] = type;
        sharedREF->ValveCtrlMode_Command[LAR] = type;
    } else {
        sharedREF->ValveCtrlMode_Command[Target] = type;
    }
}



// ========================================================================

void RBCMD_EncZero(){
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(_BN == -1) { // all joint
        for(int i = 0;i<_NO_OF_VC; i++)
        {
            _DEV_VC[i].SendGeneralMSG(ValveController_GeneralMSG_CMD_ENCZERO);
        }
    } else {
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_ENCZERO);
    }
}

void RBCMD_FindHome(void)
{
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    if(_BN == -1) {
        for(int i = 0;i<12; i++)
        {
            FindHomeOperation(i);
            sharedREF->PosOrFor_Selection[i] = JointControlMode_Position; // position control mode
        }
    }
    else if(_BN == -2){ // LIGHT Findhome process 1st stage
        FindHomeOperation(WST);
        FindHomeOperation(RAP);
        FindHomeOperation(RAR);
        FindHomeOperation(LAP);
        FindHomeOperation(LAR);
        sharedREF->PosOrFor_Selection[WST] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RAR] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LAR] = JointControlMode_Position;
    }
    else if(_BN == -3){ // LIGHT Findhome process 2nd stage
        FindHomeOperation(RHR);
        FindHomeOperation(RHY);
        FindHomeOperation(RHP);
        FindHomeOperation(RKN);
        FindHomeOperation(LHR);
        FindHomeOperation(LHY);
        FindHomeOperation(LHP);
        FindHomeOperation(LKN);
        sharedREF->PosOrFor_Selection[RHR] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RHY] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RHP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RKN] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LHR] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LHY] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LHP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LKN] = JointControlMode_Position;
    }
    else if(_BN == RAP||_BN == RAR){
        FindHomeOperation(RAP);
        FindHomeOperation(RAR);
        sharedREF->PosOrFor_Selection[RAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RAR] = JointControlMode_Position;
    }
    else if(_BN == LAP||_BN == LAR){
        FindHomeOperation(LAP);
        FindHomeOperation(LAR);
        sharedREF->PosOrFor_Selection[LAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LAR] = JointControlMode_Position;
    } else {
        FindHomeOperation(_BN);
        sharedREF->PosOrFor_Selection[_BN] = JointControlMode_Position;
    }
}

void FindHomeOperation(int _BN)
{
    for(int k=0; k<_NO_OF_AL; k++){
        sharedREF->AngleReference[k][_BN][0] = 0.0;
        sharedREF->AngVelReference[k][_BN][0] = 0.0;
        sharedREF->ActPosReference[k][_BN][0] = 0.0;
        sharedREF->ActVelReference[k][_BN][0] = 0.0;
    }
    sharedSEN->ENCODER[_BN][0].CurrentRefAngle = 0.0;
    sharedSEN->ENCODER[_BN][0].CurrentRefAngVel = 0.0;
    sharedSEN->ENCODER[_BN][0].CurrentRefActPos = 0.0;
    sharedSEN->ENCODER[_BN][0].CurrentRefActVel = 0.0;
    _DEV_VC[_BN].Joints[0].HCB_Ref.ReferencePosition = 0.0;
    _DEV_VC[_BN].Joints[0].HCB_Ref.ReferenceVelocity = 0.0;

    // Position reference sending off
    sharedREF->ValveCtrlMode[_BN] = ValveControlMode_UtilMode;

    _DEV_VC[_BN].CMD_ControlMode(22);
}


//==============================================================================
int RBCore_Initialize(void){
    cout << endl;
    FILE_LOG(logERROR) << "==========Initializing===========";

    IS_WORKING = true;

    // Shared Memory initialize
    if(RBCore_SMInitialize() == false)
        return false;

    // Load RBCore configuration file
    if(RBCore_DBInitialize() == false)
        return false;

//    // CAN initialize
////    RBCore_CANInitialize();
    if(rb_spi2can::getInstance()->IsInitialized()) {
        IS_CAN_READY = true;
    } else {
        return false;
    }

    // Real-time thread initialize
    if(RBCore_ThreadInitialize() == false)
        return false;

    // Process Manager initialize
    if(RBCore_PMInitialize() == false)
        return false;

    FILE_LOG(logERROR) << "=================================";
    cout << endl;

    sharedSEN->PODO_AL_WORKING[RBCORE_PODO_NO] = true;
    IS_WORKING = true;
    return true;
}

//---------------
int RBCore_SMInitialize(){
    shm_unlink(RBCORE_SHM_NAME_REFERENCE);
    shm_unlink(RBCORE_SHM_NAME_SENSOR);
    shm_unlink(RBCORE_SHM_NAME_COMMAND);

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME_REFERENCE, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Reference]";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_REFERENCE)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Reference]";
            return -1;
        }else{
            sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedREF == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Reference]";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Reference]";
    // =========================================================================

    // Core Shared Memory Creation [Sensor]=====================================
    shmFD = shm_open(RBCORE_SHM_NAME_SENSOR, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Sensor]";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_SENSOR)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Sensor]";
            return -1;
        }else{
            sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedSEN == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Sensor]";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Sensor]";
    // =========================================================================

    // Core Shared Memory Creation [Command]====================================
    shmFD = shm_open(RBCORE_SHM_NAME_COMMAND, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Command]";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_COMMAND)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Command]";
            return -1;
        }else{
            sharedCMD = (pRBCORE_SHM_COMMAND)mmap(0, sizeof(RBCORE_SHM_COMMAND), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedCMD == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Command]";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Command]";
    // =========================================================================

    return true;
}
//---------------
int RBCore_DBInitialize(){
    RBDataBase DB;
    DB.SetFilename("Core_Config.db");
//    DB.SetFilename("Core_Config_ValveTest.db");
    if(DB.OpenDB() == false){
        FILE_LOG(logERROR) << "Fail to load database file";
        return false;
    }

    _VERSION        = RBDataBase::_DB_GENERAL.VERSION;
    _NO_OF_AL       = RBDataBase::_DB_GENERAL.NO_OF_AL;
    _NO_OF_COMM_CH  = RBDataBase::_DB_GENERAL.NO_OF_COMM_CH;
    _NO_OF_VC       = RBDataBase::_DB_GENERAL.NO_OF_VC;
    _NO_OF_PC       = RBDataBase::_DB_GENERAL.NO_OF_PC;
    _NO_OF_FT       = RBDataBase::_DB_GENERAL.NO_OF_FT;
    _NO_OF_IMU      = RBDataBase::_DB_GENERAL.NO_OF_IMU;

    FILE_LOG(logSUCCESS) << "Core load database = OK";

    std::cout << "----------------------------------" << std::endl;
    std::cout << "     VERSION       : " << _VERSION << std::endl;
    std::cout << "     NO_OF_AL      : " << _NO_OF_AL << std::endl;
    std::cout << "     NO_OF_COMM_CH : " << _NO_OF_COMM_CH << std::endl;
    std::cout << "     NO_OF_VC      : " << _NO_OF_VC << std::endl;
    std::cout << "     NO_OF_PC      : " << _NO_OF_PC << std::endl;
    std::cout << "     NO_OF_FT      : " << _NO_OF_FT << std::endl;
    std::cout << "     NO_OF_IMU     : " << _NO_OF_IMU << std::endl;
    std::cout << "----------------------------------" << std::endl;

    for(int i=0; i<_NO_OF_VC; i++)   _DEV_VC[i].GetDBData(RBDataBase::_DB_VC[i]);
    for(int i=0; i<_NO_OF_PC; i++)   _DEV_PC[i].GetDBData(RBDataBase::_DB_PC[i]);
    for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBBoard_GetDBData(RBDataBase::_DB_FT[i]);
    for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBBoard_GetDBData(RBDataBase::_DB_IMU[i]);

    return true;
}
//---------------
int RBCore_CANInitialize(){
//    canHandler = new RBCAN(_NO_OF_COMM_CH);

//    if(canHandler->IsWorking() == false){
//        IS_CAN_READY = false;
//        return false;
//    }else{
//        _CANOUT_ENABLED = true;

//        for(int i=0; i<_NO_OF_VC; i++)   _DEV_VC[i].AddCANMailBox_RCVDATA(); // Hydraulic Actuator Controller
////        for(int i=0; i<_NO_OF_PC; i++)   _DEV_PC[i].AddCANMailBox_RCVDATA(); // Hydraulic Pump Controller
////        for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBFT_AddCANMailBox();
////        for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBIMU_AddCANMailBox();
////        for(int i=0; i<_NO_OF_SP; i++)   _DEV_SP[i].RBSP_AddCANMailBox();
////        for(int i=0; i<_NO_OF_OF; i++)   _DEV_OF[i].RBOF_AddCANMailBox();
//        IS_CAN_READY = true;
//        return true;
//    }
}
//---------------
int RBCore_LANInitialize(){
    lanHandler = new RBRawLAN();
    if(lanHandler->RBLanOpen(8888) == false){
        FILE_LOG(logSUCCESS) << "qq";
        return false;
    }
    FILE_LOG(logSUCCESS) << "lan success";
    return true;
}
//---------------
int RBCore_ThreadInitialize(){

    pthread_t thread_Daemon;
    if(generate_rt_thread(thread_Daemon, RBCore_RTThreadCon, "Daemon_thread", 2, 95, NULL)) {
        FILE_LOG(logSUCCESS) << "Core real-time thread start = OK";
        return true;
    } else {
        FILE_LOG(logERROR) << "Fail to create core real-time thread";
        return false;
    }

//    if(rt_task_create(&rtTaskCon, "RBCORE_TASK", 0, 99, 0) == 0){
//        cpu_set_t aCPU;
//        CPU_ZERO(&aCPU);
//        CPU_SET(0, &aCPU);
//        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
//            FILE_LOG(logWARNING) << "Core real-time thread set affinity CPU failed..";
//        }
//        if(rt_task_start(&rtTaskCon, &RBCore_RTThreadCon, NULL) == 0){
//            FILE_LOG(logSUCCESS) << "Core real-time thread start = OK";
//        }else{
//            FILE_LOG(logERROR) << "Core real-time thread start = FAIL";
//            return false;
//        }
//    }else{
//        FILE_LOG(logERROR) << "Fail to create core real-time thread";
//        return false;
//    }
    return true;
}
//---------------
int RBCore_PMInitialize(){
    pmHandler = new RBProcessManager();

    int ret = pmHandler->OpenAL(1);
    if(ret == -99){
        IS_CHILD = true;
        IS_WORKING = false;
        return false;
    }
    return true;
}
//---------------
int RBCore_Termination(){
    if(IS_CHILD)
        return true;

//    rt_task_delete(&rtTaskCon);
    shm_unlink(RBCORE_SHM_NAME_REFERENCE);
    shm_unlink(RBCORE_SHM_NAME_SENSOR);
    shm_unlink(RBCORE_SHM_NAME_COMMAND);
    FILE_LOG(logERROR) << "RBCore will be terminated..";
    //    exit(0);
    return true;
}

//==============================================================================
//==============================================================================
//==============================================================================

double dRAP_oo = 0.0;
double dRAP_o = 0.0;
double dRAR_oo = 0.0;
double dRAR_o = 0.0;
double dLAP_oo = 0.0;
double dLAP_o = 0.0;
double dLAR_oo = 0.0;
double dLAR_o = 0.0;

void ReadBoardInfo()
{
    for(int i=0; i<_NO_OF_VC; i++) {
        sharedSEN->ENCODER[i][0].HCB_Info = _DEV_VC[i].Joints[0].HCB_Info;
    }
}


void ReadActuatorData()
{
    for(int i=0; i<_NO_OF_VC; i++)
    {
        if(_DEV_VC[i].Joints[0].HCB_Info.REQUEST_ONOFF_PosVel)
        {
            for(int j=0; j<MAX_JOINT; j++)
            {
                sharedSEN->ENCODER[i][j].CurrentActPos = _DEV_VC[i].Joints[j].HCB_Data.CurrentPosition;
                sharedSEN->ENCODER[i][j].CurrentActVel = _DEV_VC[i].Joints[j].HCB_Data.CurrentVelocity;
                sharedSEN->ENCODER[i][j].CurrentActForce = _DEV_VC[i].Joints[j].HCB_Data.CurrentForce;

//                if(_DEV_VC[i].ACTUATOR_TYPE == "LIN")
//                {
//                    // ACTUATOR_TYPE : Linear Actuator
//                    if(_DEV_VC[i].BOARD_NAME == "HCB_RHP"||_DEV_VC[i].BOARD_NAME == "HCB_LHP") {
//                        double x = sharedSEN->ENCODER[i][j].CurrentActPos*0.001f;
//                        double dx = sharedSEN->ENCODER[i][j].CurrentActVel*0.001f;
//                        double F = sharedSEN->ENCODER[i][j].CurrentActForce;
//                        double theta, dtheta, T;
//                        Actuator2Joint_HipPitch(x, dx, F, theta, dtheta, T);
//                        sharedSEN->ENCODER[i][j].CurrentAngle  = theta*R2D;
//                        sharedSEN->ENCODER[i][j].CurrentAngVel = dtheta*R2D;
//                        sharedSEN->ENCODER[i][j].CurrentTorque = T;
//                    } else if(_DEV_VC[i].BOARD_NAME == "HCB_RKN"||_DEV_VC[i].BOARD_NAME == "HCB_LKN") {
//                        double x = sharedSEN->ENCODER[i][j].CurrentActPos*0.001f; // mm >> m
//                        double dx = sharedSEN->ENCODER[i][j].CurrentActVel*0.001f; // mm >> m
//                        double F = sharedSEN->ENCODER[i][j].CurrentActForce;
//                        double theta, dtheta, T;
//                        Actuator2Joint_Knee(x, dx, F, theta, dtheta, T);
//                        sharedSEN->ENCODER[i][j].CurrentAngle  = theta*R2D;
//                        sharedSEN->ENCODER[i][j].CurrentAngVel = dtheta*R2D;
//                        sharedSEN->ENCODER[i][j].CurrentTorque = T;
//                    } else if(_DEV_VC[i].BOARD_NAME == "HCB_RA_1") {
//                        double theta_init_guess = 40.0f*D2R; // sharedSEN->ENCODER[RAP][0].CurrentAngle*D2R; //
//                        double phi_init_guess = 0.0f*D2R; // sharedSEN->ENCODER[RAR][0].CurrentAngle*D2R; //
////                        double theta_init_guess = sharedSEN->ENCODER[RAP][0].CurrentAngle*D2R; //
////                        double phi_init_guess = sharedSEN->ENCODER[RAR][0].CurrentAngle*D2R; //
//                        double x1 = sharedSEN->ENCODER[RAP][0].CurrentActPos*0.001f;
//                        double x2 = sharedSEN->ENCODER[RAR][0].CurrentActPos*0.001f;
//                        double dx1 = sharedSEN->ENCODER[RAP][0].CurrentActVel*0.001f;
//                        double dx2 = sharedSEN->ENCODER[RAR][0].CurrentActVel*0.001f;
//                        double F1 = sharedSEN->ENCODER[RAP][0].CurrentActForce;
//                        double F2 = sharedSEN->ENCODER[RAR][0].CurrentActForce;

//                        double theta_new, phi_new;
//                        double dtheta, dphi;
//                        double T_pitch, T_roll;
//                        if(Actuator2Joint_Ankle(theta_init_guess,phi_init_guess,
//                                             x1,x2,dx1,dx2,F1,F2,
//                                             theta_new,phi_new,dtheta,dphi,T_pitch, T_roll))
//                        {
//                            sharedSEN->ENCODER[RAP][0].CurrentAngle = theta_new*R2D;
//                            sharedSEN->ENCODER[RAR][0].CurrentAngle = phi_new*R2D;
//                            sharedSEN->ENCODER[RAP][0].CurrentAngVel = dtheta*R2D;
//                            sharedSEN->ENCODER[RAR][0].CurrentAngVel = dphi*R2D;
//                            sharedSEN->ENCODER[RAP][0].CurrentTorque = T_pitch;
//                            sharedSEN->ENCODER[RAR][0].CurrentTorque = T_roll;
//                        }
//                    } else if(_DEV_VC[i].BOARD_NAME == "HCB_RA_2") {
//                        // "HCB_RA_2" Skipped
//                    } else if(_DEV_VC[i].BOARD_NAME == "HCB_LA_1") {
//                        double theta_init_guess = 40.0f*D2R;// sharedSEN->ENCODER[LAP][0].CurrentAngle*D2R; //
//                        double phi_init_guess = 0.0f*D2R;// sharedSEN->ENCODER[LAR][0].CurrentAngle*D2R; //
////                        double theta_init_guess = sharedSEN->ENCODER[LAP][0].CurrentAngle*D2R; //
////                        double phi_init_guess = sharedSEN->ENCODER[LAR][0].CurrentAngle*D2R; //
//                        double x1 = sharedSEN->ENCODER[LAP][0].CurrentActPos*0.001f;
//                        double x2 = sharedSEN->ENCODER[LAR][0].CurrentActPos*0.001f;
//                        double dx1 = sharedSEN->ENCODER[LAP][0].CurrentActVel*0.001f;
//                        double dx2 = sharedSEN->ENCODER[LAR][0].CurrentActVel*0.001f;
//                        double F1 = sharedSEN->ENCODER[LAP][0].CurrentActForce;
//                        double F2 = sharedSEN->ENCODER[LAR][0].CurrentActForce;

//                        double theta_new, phi_new;
//                        double dtheta, dphi;
//                        double T_pitch, T_roll;
//                        if(Actuator2Joint_Ankle(theta_init_guess,phi_init_guess,
//                                                x1,x2,dx1,dx2,F1,F2,
//                                                theta_new,phi_new,dtheta,dphi,T_pitch, T_roll))
//                        {
//                            sharedSEN->ENCODER[LAP][0].CurrentAngle = theta_new*R2D;
//                            sharedSEN->ENCODER[LAR][0].CurrentAngle = phi_new*R2D;
//                            sharedSEN->ENCODER[LAP][0].CurrentAngVel = dtheta*R2D;
//                            sharedSEN->ENCODER[LAR][0].CurrentAngVel = dphi*R2D;
//                            sharedSEN->ENCODER[LAP][0].CurrentTorque = T_pitch;
//                            sharedSEN->ENCODER[LAR][0].CurrentTorque = T_roll;
//                        }
//                    } else if(_DEV_VC[i].BOARD_NAME == "HCB_LA_2") {
//                        // "HCB_LA_2" Skipped
//                    } else {
//                        FILE_LOG(logERROR) << "Not Cylinder TYPE!!";
//                    }
//                } else if (_DEV_VC[i].ACTUATOR_TYPE == "ROT") {
//                    // ACTUATOR_TYPE : Rotary Actuator
//                    sharedSEN->ENCODER[i][j].CurrentAngle = sharedSEN->ENCODER[i][j].CurrentActPos;
//                    sharedSEN->ENCODER[i][j].CurrentAngVel = sharedSEN->ENCODER[i][j].CurrentActVel;
//                    sharedSEN->ENCODER[i][j].CurrentTorque = sharedSEN->ENCODER[i][j].CurrentActForce;
//                } else {
//                    FILE_LOG(logERROR) << "Not Defined ACTUATOR TYPE!!";
//                }
            }
        }
    }
}

void ReadValvePosnPWM()
{
    for(int i=0; i<_NO_OF_VC; i++)
    {
        if(_DEV_VC[i].Joints[0].HCB_Info.REQUEST_ONOFF_ValvePosnPWM)
        {
            for(int j=0; j<MAX_JOINT; j++){
                sharedSEN->ENCODER[i][j].CurrentValvePos    = _DEV_VC[i].Joints[j].HCB_Data.CurrentValvePos;
                sharedSEN->ENCODER[i][j].CurrentValvePosRef = _DEV_VC[i].Joints[j].HCB_Data.CurrentValvePosRef;
                sharedSEN->ENCODER[i][j].CurrentPWM         = _DEV_VC[i].Joints[j].HCB_Data.CurrentPWM;
            }
        }
    }
}

void ReadValvePressure()
{
    for(int i=0; i<_NO_OF_VC; i++)
    {
        if(_DEV_VC[i].Joints[0].HCB_Info.REQUEST_ONOFF_Pressure)
        {
            for(int j=0; j<MAX_JOINT; j++){
                sharedSEN->ENCODER[i][j].CurrentPressureA = _DEV_VC[i].Joints[j].HCB_Data.CurrentPressureA;
                sharedSEN->ENCODER[i][j].CurrentPressureB = _DEV_VC[i].Joints[j].HCB_Data.CurrentPressureB;
            }
        }
    }
}

void ReadValveOtherData()
{
    for(int i=0; i<_NO_OF_VC; i++)
    {
        if(_DEV_VC[i].Joints[0].HCB_Info.REQUEST_ONOFF_OtherInfo)
        {
            for(int j=0; j<MAX_JOINT; j++){
                sharedSEN->ENCODER[i][j].CurrentData1 = _DEV_VC[i].Joints[j].HCB_Data.CurrentTempData1;
                sharedSEN->ENCODER[i][j].CurrentData2 = _DEV_VC[i].Joints[j].HCB_Data.CurrentTempData2;
                sharedSEN->ENCODER[i][j].CurrentData3 = _DEV_VC[i].Joints[j].HCB_Data.CurrentTempData3;
                sharedSEN->ENCODER[i][j].CurrentData4 = _DEV_VC[i].Joints[j].HCB_Data.CurrentTempData4;
            }
        }
    }
}

void ReadValveAlart()
{
    for(int i=0; i<_NO_OF_VC; i++)
    {

    }
}

float local_wx_o,local_ax_o;
float local_wy_o,local_ay_o;
float local_wz_o,local_az_o;
float local_wx_oo,local_ax_oo;
float local_wy_oo,local_ay_oo;
float local_wz_oo,local_az_oo;

float local_wx_f,local_ax_f;
float local_wy_f,local_ay_f;
float local_wz_f,local_az_f;
float local_wx_fo,local_ax_fo;
float local_wy_fo,local_ay_fo;
float local_wz_fo,local_az_fo;
float local_wx_foo,local_ax_foo;
float local_wy_foo,local_ay_foo;
float local_wz_foo,local_az_foo;

void ReadNewIMU(void){

    if(_DEV_IMU[0].ConnectionStatus == true) {

        float local_wx = _DEV_IMU[0].wx_local;
        float local_wy = _DEV_IMU[0].wy_local;
        float local_wz = _DEV_IMU[0].wz_local;
        float local_ax = _DEV_IMU[0].ax_local;
        float local_ay = _DEV_IMU[0].ay_local;
        float local_az = _DEV_IMU[0].az_local;

        //        // Low pass filter (or notch filter) for pump vibration
        //        if(sharedSEN->PUMP[0].CurrentVelocity < 600.0) {
        //            local_wx_f = local_wx;
        //            local_wy_f = local_wy;
        //            local_wz_f = local_wz;
        //            local_ax_f = local_ax;
        //            local_ay_f = local_ay;
        //            local_az_f = local_az;
        //        } else {
        //            double wdt_notch = 2.0*3.141592*sharedSEN->PUMP[0].CurrentVelocity/60.0*(double)RT_TIMER_PERIOD_MS/1000.0;
        //            double Q_notch = 0.3;
        //            double a1_notch = 1.0 + wdt_notch/Q_notch + wdt_notch*wdt_notch;
        //            double a2_notch = -2.0 - wdt_notch/Q_notch;
        //            double a3_notch = 1.0;
        //            double b1_notch = 1.0 + wdt_notch*wdt_notch;
        //            double b2_notch = -2.0;
        //            double b3_notch = 1.0;

        //            local_wx_f = (b1_notch*local_wx + b2_notch*local_wx_o + b3_notch*local_wx_oo - a2_notch*local_wx_fo - a3_notch*local_wx_foo)/a1_notch;
        //            local_wy_f = (b1_notch*local_wy + b2_notch*local_wy_o + b3_notch*local_wy_oo - a2_notch*local_wy_fo - a3_notch*local_wy_foo)/a1_notch;
        //            local_wz_f = (b1_notch*local_wz + b2_notch*local_wz_o + b3_notch*local_wz_oo - a2_notch*local_wz_fo - a3_notch*local_wz_foo)/a1_notch;
        //            local_ax_f = (b1_notch*local_ax + b2_notch*local_ax_o + b3_notch*local_ax_oo - a2_notch*local_ax_fo - a3_notch*local_ax_foo)/a1_notch;
        //            local_ay_f = (b1_notch*local_ay + b2_notch*local_ay_o + b3_notch*local_ay_oo - a2_notch*local_ay_fo - a3_notch*local_ay_foo)/a1_notch;
        //            local_az_f = (b1_notch*local_az + b2_notch*local_az_o + b3_notch*local_az_oo - a2_notch*local_az_fo - a3_notch*local_az_foo)/a1_notch;
        //        }

        //        double w_cut = 2.0*PI*1.0;
        //        double SYS_FREQ = RT_TIMER_FREQ;
        //        double alpha1 = -(SYS_FREQ*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
        //        double alpha2 = (2.0*SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
        //        double alpha3 = (w_cut*w_cut)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);

        //        local_wx_f = alpha1*local_wx_oo + alpha2*local_wx_o + alpha3*local_wx;
        //        local_wy_f = alpha1*local_wy_oo + alpha2*local_wy_o + alpha3*local_wy;
        //        local_wz_f = alpha1*local_wz_oo + alpha2*local_wz_o + alpha3*local_wz;
        //        local_ax_f = alpha1*local_ax_oo + alpha2*local_ax_o + alpha3*local_ax;
        //        local_ay_f = alpha1*local_ay_oo + alpha2*local_ay_o + alpha3*local_ay;
        //        local_az_f = alpha1*local_az_oo + alpha2*local_az_o + alpha3*local_az;

        double fcut_wp = 12.0;
        double alpha_wp = 1.0/(1.0+2.0*PI*fcut_wp*(double)RT_TIMER_PERIOD_US/1000000.0);

        local_wx_f = alpha_wp*local_wx_f + (1.0-alpha_wp)*local_wx;
        local_wy_f = alpha_wp*local_wy_f + (1.0-alpha_wp)*local_wy;
        local_wz_f = alpha_wp*local_wz_f + (1.0-alpha_wp)*local_wz;
        local_ax_f = alpha_wp*local_ax_f + (1.0-alpha_wp)*local_ax;
        local_ay_f = alpha_wp*local_ay_f + (1.0-alpha_wp)*local_ay;
        local_az_f = alpha_wp*local_az_f + (1.0-alpha_wp)*local_az;

        local_wx_oo = local_wx_o;
        local_wy_oo = local_wy_o;
        local_wz_oo = local_wz_o;
        local_ax_oo = local_ax_o;
        local_ay_oo = local_ay_o;
        local_az_oo = local_az_o;
        local_wx_o = local_wx;
        local_wy_o = local_wy;
        local_wz_o = local_wz;
        local_ax_o = local_ax;
        local_ay_o = local_ay;
        local_az_o = local_az;

        local_wx_foo = local_wx_fo;
        local_wy_foo = local_wy_fo;
        local_wz_foo = local_wz_fo;
        local_ax_foo = local_ax_fo;
        local_ay_foo = local_ay_fo;
        local_az_foo = local_az_fo;
        local_wx_fo = local_wx_f;
        local_wy_fo = local_wy_f;
        local_wz_fo = local_wz_f;
        local_ax_fo = local_ax_f;
        local_ay_fo = local_ay_f;
        local_az_fo = local_az_f;


        // reversed IMU with sponge (210830)
        sharedSEN->IMU[0].Wx_B      = (double)local_wy_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Wy_B      = (double)local_wx_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Wz_B      = (double)-local_wz_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Ax_B      = (double)local_ay_f*9.81; // Unit : (normalized g) > m/s^2
        sharedSEN->IMU[0].Ay_B      = (double)local_ax_f*9.81; // Unit : (normalized g) > m/s^2
        sharedSEN->IMU[0].Az_B      = (double)-local_az_f*9.81; // Unit : (normalized g) > m/s^2

        // for LIGHT2
        //        sharedSEN->IMU[0].Wx_B      = (double)local_wy_f*D2R; // Unit : deg/s > rad/s
        //        sharedSEN->IMU[0].Wy_B      = (double)-local_wx_f*D2R; // Unit : deg/s > rad/s
        //        sharedSEN->IMU[0].Wz_B      = (double)local_wz_f*D2R; // Unit : deg/s > rad/s
        //        sharedSEN->IMU[0].Ax_B      = (double)local_ay_f*9.81; // Unit : (normalized g) > m/s^2
        //        sharedSEN->IMU[0].Ay_B      = (double)-local_ax_f*9.81; // Unit : (normalized g) > m/s^2
        //        sharedSEN->IMU[0].Az_B      = (double)local_az_f*9.81; // Unit : (normalized g) > m/s^2

        // Invarient Extended Kalman Filter
        Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
        Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
        InEKF_IMU.Update(Uw,Ua);
        Matrix3_InEKF R = InEKF_IMU.R;

        // IMU Offset Compensation
        double roll_off = _DEV_IMU[0].ROLL_OFFSET;
        double pitch_off = _DEV_IMU[0].PITCH_OFFSET;
        Matrix3_InEKF R_Offset_Roll(1.0,            0.0,            0.0,
                                    0.0,  cos(roll_off), -sin(roll_off),
                                    0.0,  sin(roll_off),  cos(roll_off));
        Matrix3_InEKF R_Offset_Pitch( cos(pitch_off), 0.0, sin(pitch_off),
                                      0.0, 1.0,            0.0,
                                      -sin(pitch_off), 0.0, cos(pitch_off));
        R = R_Offset_Roll.Transpose()*R_Offset_Pitch.Transpose()*R;

        // Projection to Yaw rotation angle zero
        bool Flag_YawZero = true;
        if(Flag_YawZero) {
            if(fabs(sqrt(R.M[1][0]*R.M[1][0]+R.M[0][0]*R.M[0][0]))<1e-6) {
                if(R.M[0][0] < 0.0) {
                    sharedSEN->IMU[0].Roll = 0.0;
                    sharedSEN->IMU[0].Pitch = 90.0*D2R;
                    sharedSEN->IMU[0].Yaw = 0.0;
                    Matrix3_InEKF R_yawzero(0.0, 0.0, 1.0,
                                            0.0, 1.0, 0.0,
                                            1.0, 0.0, 0.0);

                    Vector3_InEKF W_G = R_yawzero*Uw;
                    sharedSEN->IMU[0].Wx_G = W_G.v[0];
                    sharedSEN->IMU[0].Wy_G = W_G.v[1];
                    sharedSEN->IMU[0].Wz_G = W_G.v[2];

                    Vector3_InEKF A_G = R_yawzero*Ua;
                    sharedSEN->IMU[0].Ax_G = A_G.v[0];
                    sharedSEN->IMU[0].Ay_G = A_G.v[1];
                    sharedSEN->IMU[0].Az_G = A_G.v[2];
                } else {
                    sharedSEN->IMU[0].Roll = 0.0;
                    sharedSEN->IMU[0].Pitch = -90.0*D2R;
                    sharedSEN->IMU[0].Yaw = 0.0;
                    Matrix3_InEKF R_yawzero(0.0, 0.0, -1.0,
                                            0.0, 1.0, 0.0,
                                            -1.0, 0.0, 0.0);

                    Vector3_InEKF W_G = R_yawzero*Uw;
                    sharedSEN->IMU[0].Wx_G = W_G.v[0];
                    sharedSEN->IMU[0].Wy_G = W_G.v[1];
                    sharedSEN->IMU[0].Wz_G = W_G.v[2];

                    Vector3_InEKF A_G = R_yawzero*Ua;
                    sharedSEN->IMU[0].Ax_G = A_G.v[0];
                    sharedSEN->IMU[0].Ay_G = A_G.v[1];
                    sharedSEN->IMU[0].Az_G = A_G.v[2];
                }
            } else {
                double psi = atan2(R.M[1][0], R.M[0][0]);   // z-axis (Yaw)
                //            double theta = -asin(R.M[2][0]);            // y'-axis (Pitch)
                //            double phi = atan2(R.M[2][1], R.M[2][2]);   // x''-axis (Roll)
                Matrix3_InEKF R_yaw(cos(psi), -sin(psi), 0.0,
                                    sin(psi), cos(psi) , 0.0,
                                    0.0     , 0.0      , 1.0);
                Matrix3_InEKF R_yawzero = R_yaw.Transpose()*R;

                sharedSEN->IMU[0].Q[0] = sqrt(1.0+R_yawzero.M[0][0]+R_yawzero.M[1][1]+R_yawzero.M[2][2])/2.0;
                sharedSEN->IMU[0].Q[1] = (R_yawzero.M[2][1]-R_yawzero.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
                sharedSEN->IMU[0].Q[2] = (R_yawzero.M[0][2]-R_yawzero.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
                sharedSEN->IMU[0].Q[3] = (R_yawzero.M[1][0]-R_yawzero.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

                sharedSEN->IMU[0].Roll = atan2(-R_yawzero.M[1][2],R_yawzero.M[1][1]);
                sharedSEN->IMU[0].Pitch = atan2(-R_yawzero.M[2][0],R_yawzero.M[0][0]);
                sharedSEN->IMU[0].Yaw = 0.0;

                Vector3_InEKF W_G = R_yawzero*Uw;
                sharedSEN->IMU[0].Wx_G = W_G.v[0];
                sharedSEN->IMU[0].Wy_G = W_G.v[1];
                sharedSEN->IMU[0].Wz_G = W_G.v[2];

                Vector3_InEKF A_G = R_yawzero*Ua;
                sharedSEN->IMU[0].Ax_G = A_G.v[0];
                sharedSEN->IMU[0].Ay_G = A_G.v[1];
                sharedSEN->IMU[0].Az_G = A_G.v[2];
            }
        } else {
            sharedSEN->IMU[0].Q[0] = sqrt(1.0+R.M[0][0]+R.M[1][1]+R.M[2][2])/2.0;
            sharedSEN->IMU[0].Q[1] = (R.M[2][1]-R.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[2] = (R.M[0][2]-R.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[3] = (R.M[1][0]-R.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

            double q0 = sharedSEN->IMU[0].Q[0];
            double q1 = sharedSEN->IMU[0].Q[1];
            double q2 = sharedSEN->IMU[0].Q[2];
            double q3 = sharedSEN->IMU[0].Q[3];
            sharedSEN->IMU[0].Yaw   = atan2(2.0*(q1*q2 + q0*q3), 1.0-2.0*(q2*q2 + q3*q3)); // z-axis
            sharedSEN->IMU[0].Pitch = -asin(2.0*(q1*q3 - q0*q2));                          // y'-axis
            sharedSEN->IMU[0].Roll  = atan2(2.0*(q2*q3 + q0*q1), 1.0-2.0*(q1*q1 + q2*q2)); // x''-axis

            Vector3_InEKF W_G = R*Uw;
            sharedSEN->IMU[0].Wx_G = W_G.v[0];
            sharedSEN->IMU[0].Wy_G = W_G.v[1];
            sharedSEN->IMU[0].Wz_G = W_G.v[2];

            Vector3_InEKF A_G = R*Ua;
            sharedSEN->IMU[0].Ax_G = A_G.v[0];
            sharedSEN->IMU[0].Ay_G = A_G.v[1];
            sharedSEN->IMU[0].Az_G = A_G.v[2];
        }
    }
}


void ReadFT(){
    for(int i=0; i<_NO_OF_FT; i++)
    {
        if(_DEV_FT[i].ConnectionStatus == true) {
            sharedSEN->FT[i].Fx     = _DEV_FT[i].FX;
            sharedSEN->FT[i].Fy     = -_DEV_FT[i].FY;
            sharedSEN->FT[i].Fz     = -_DEV_FT[i].FZ;
            sharedSEN->FT[i].Mx     = -_DEV_FT[i].MX;
            sharedSEN->FT[i].My     = -_DEV_FT[i].MY;
            sharedSEN->FT[i].Mz     = _DEV_FT[i].MZ;
        }
    }
}

void ReadPumpData()
{
    double alpha_velo = 1.0/(1.0+RT_TIMER_FREQ/(2.0*3.141592*30.0));
    double alpha_pres = 1.0/(1.0+RT_TIMER_FREQ/(2.0*3.141592*5.0));
    double alpha_temp = 1.0/(1.0+RT_TIMER_FREQ/(2.0*3.141592*10.0));
    sharedSEN->PUMP[0].CurrentVelocity      = (1.0-alpha_velo)*sharedSEN->PUMP[0].CurrentVelocity + alpha_velo*_DEV_PC[0].CurrentVelocity;
    sharedSEN->PUMP[0].CurrentPressure      = (1.0-alpha_pres)*sharedSEN->PUMP[0].CurrentPressure + alpha_pres*_DEV_PC[0].CurrentPressure;
    sharedSEN->PUMP[0].CurrentTemperature   = (1.0-alpha_temp)*sharedSEN->PUMP[0].CurrentTemperature + alpha_temp*_DEV_PC[0].CurrentTemperature;

    sharedSEN->PUMP[0].CurrentHydraPower    = (sharedSEN->PUMP[0].CurrentPressure*100.0)*(sharedSEN->PUMP[0].CurrentVelocity/3000.0*14.0/60.0);
    double temp_HydraPower = 0.0;
    for(int i=0;i<MAX_VC;i++) {
        temp_HydraPower += sharedSEN->ENCODER[i][0].CurrentActVel*D2R*sharedSEN->ENCODER[i][0].CurrentActForce;
    }
    sharedSEN->PUMP[0].CurrentMechaPower    = temp_HydraPower;
}

void ReadSimulationData()
{
    // Joint angle and torque sensor update
    for(int i=0; i<_NO_OF_VC; i++) {
        for(int j=0; j<MAX_JOINT; j++) {
            sharedSEN->ENCODER[i][j].CurrentAngle = sharedREF->Simulation_AngleSensor[i];
            sharedSEN->ENCODER[i][j].CurrentAngVel = sharedREF->Simulation_VelocitySensor[i];
            sharedSEN->ENCODER[i][j].CurrentTorque = sharedREF->Simulation_TorqueSensor[i];
        }
    }
    sharedSEN->IMU[0].Wx_B  = sharedREF->Simulation_IMU[0].Wx_B;
    sharedSEN->IMU[0].Wy_B  = sharedREF->Simulation_IMU[0].Wy_B;
    sharedSEN->IMU[0].Wz_B  = sharedREF->Simulation_IMU[0].Wz_B;
    sharedSEN->IMU[0].Ax_B  = sharedREF->Simulation_IMU[0].Ax_B;
    sharedSEN->IMU[0].Ay_B  = sharedREF->Simulation_IMU[0].Ay_B;
    sharedSEN->IMU[0].Az_B  = sharedREF->Simulation_IMU[0].Az_B;

    // By Simulation Data
    double q0 = sharedREF->Simulation_IMU[0].Q[0];
    double q1 = sharedREF->Simulation_IMU[0].Q[1];
    double q2 = sharedREF->Simulation_IMU[0].Q[2];
    double q3 = sharedREF->Simulation_IMU[0].Q[3];

    double psi = atan2(2 * (q1*q2 + q0*q3), 1 - 2 * (q2*q2 + q3*q3));    // z-axis
    double theta = -1 * asin(2 * (q1*q3 - q0*q2));                       // y'-axis
    double phi = atan2(2 * (q2*q3 + q0*q1), 1 - 2 * (q1*q1 + q2*q2));    // x''-axis

    Matrix3_InEKF R; // Pelvis frame orientation w.r.t the global frame.
    R.M[0][0] = 1.0-2.0*(q2*q2+q3*q3);
    R.M[0][1] = 2.0*(q1*q2-q3*q0);
    R.M[0][2] = 2.0*(q0*q2+q1*q3);
    R.M[1][0] = 2.0*(q1*q2+q3*q0);
    R.M[1][1] = 1.0-2.0*(q1*q1+q3*q3);
    R.M[1][2] = 2.0*(q2*q3-q0*q1);
    R.M[2][0] = 2.0*(q1*q3-q0*q2);
    R.M[2][1] = 2.0*(q0*q1+q2*q3);
    R.M[2][2] = 1.0-2.0*(q2*q2+q1*q1);

    // Projection to Yaw rotation zero
    bool Flag_YawZero = true;
    if(Flag_YawZero) {
        if(fabs(sqrt(R.M[1][0]*R.M[1][0]+R.M[0][0]*R.M[0][0]))<1e-6) {
            if(R.M[0][0] < 0.0) {
                sharedSEN->IMU[0].Roll = 0.0;
                sharedSEN->IMU[0].Pitch = 90.0*R2D;
                sharedSEN->IMU[0].Yaw = 0.0;
                Matrix3_InEKF R_yawzero(0.0, 0.0, 1.0,
                                        0.0, 1.0, 0.0,
                                        1.0, 0.0, 0.0);

                Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
                Vector3_InEKF W_G = R_yawzero*Uw;
                sharedSEN->IMU[0].Wx_G = W_G.v[0];
                sharedSEN->IMU[0].Wy_G = W_G.v[1];
                sharedSEN->IMU[0].Wz_G = W_G.v[2];

                Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
                Vector3_InEKF A_G = R_yawzero*Ua;
                sharedSEN->IMU[0].Ax_G = A_G.v[0];
                sharedSEN->IMU[0].Ay_G = A_G.v[1];
                sharedSEN->IMU[0].Az_G = A_G.v[2];
            } else {
                sharedSEN->IMU[0].Roll = 0.0;
                sharedSEN->IMU[0].Pitch = -90.0*R2D;
                sharedSEN->IMU[0].Yaw = 0.0;
                Matrix3_InEKF R_yawzero(0.0, 0.0, -1.0,
                                        0.0, 1.0, 0.0,
                                        -1.0, 0.0, 0.0);

                Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
                Vector3_InEKF W_G = R_yawzero*Uw;
                sharedSEN->IMU[0].Wx_G = W_G.v[0];
                sharedSEN->IMU[0].Wy_G = W_G.v[1];
                sharedSEN->IMU[0].Wz_G = W_G.v[2];

                Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
                Vector3_InEKF A_G = R_yawzero*Ua;
                sharedSEN->IMU[0].Ax_G = A_G.v[0];
                sharedSEN->IMU[0].Ay_G = A_G.v[1];
                sharedSEN->IMU[0].Az_G = A_G.v[2];
            }
        } else {
            double psi = atan2(R.M[1][0], R.M[0][0]);   // z-axis (Yaw)
            //            double theta = -asin(R.M[2][0]);            // y'-axis (Pitch)
            //            double phi = atan2(R.M[2][1], R.M[2][2]);   // x''-axis (Roll)
            Matrix3_InEKF R_yaw(cos(psi), -sin(psi), 0.0,
                                sin(psi), cos(psi) , 0.0,
                                0.0     , 0.0      , 1.0);
            Matrix3_InEKF R_yawzero = R_yaw.Transpose()*R;

            sharedSEN->IMU[0].Q[0] = sqrt(1.0+R_yawzero.M[0][0]+R_yawzero.M[1][1]+R_yawzero.M[2][2])/2.0;
            sharedSEN->IMU[0].Q[1] = (R_yawzero.M[2][1]-R_yawzero.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[2] = (R_yawzero.M[0][2]-R_yawzero.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[3] = (R_yawzero.M[1][0]-R_yawzero.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

            sharedSEN->IMU[0].Roll = atan2(-R_yawzero.M[1][2],R_yawzero.M[1][1]);
            sharedSEN->IMU[0].Pitch = atan2(-R_yawzero.M[2][0],R_yawzero.M[0][0]);
            sharedSEN->IMU[0].Yaw = 0.0;

            Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
            Vector3_InEKF W_G = R_yawzero*Uw;
            sharedSEN->IMU[0].Wx_G = W_G.v[0];
            sharedSEN->IMU[0].Wy_G = W_G.v[1];
            sharedSEN->IMU[0].Wz_G = W_G.v[2];

            Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
            Vector3_InEKF A_G = R_yawzero*Ua;
            sharedSEN->IMU[0].Ax_G = A_G.v[0];
            sharedSEN->IMU[0].Ay_G = A_G.v[1];
            sharedSEN->IMU[0].Az_G = A_G.v[2];
        }
    } else {
        sharedSEN->IMU[0].Q[0] = q0;
        sharedSEN->IMU[0].Q[1] = q1;
        sharedSEN->IMU[0].Q[2] = q2;
        sharedSEN->IMU[0].Q[3] = q3;

        sharedSEN->IMU[0].Yaw = psi;
        sharedSEN->IMU[0].Pitch = theta;
        sharedSEN->IMU[0].Roll = phi;

        Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
        Vector3_InEKF W_G = R*Uw;
        sharedSEN->IMU[0].Wx_G = W_G.v[0];
        sharedSEN->IMU[0].Wy_G = W_G.v[1];
        sharedSEN->IMU[0].Wz_G = W_G.v[2];

        Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
        Vector3_InEKF A_G = R*Ua;
        sharedSEN->IMU[0].Ax_G = A_G.v[0];
        sharedSEN->IMU[0].Ay_G = A_G.v[1];
        sharedSEN->IMU[0].Az_G = A_G.v[2];
    }

    // Ankle FT Sensor
    sharedSEN->FT[0].Mx = sharedREF->Simulation_FT[0].Mx;
    sharedSEN->FT[0].My = sharedREF->Simulation_FT[0].My;
    sharedSEN->FT[0].Mz = sharedREF->Simulation_FT[0].Mz;
    sharedSEN->FT[0].Fx = sharedREF->Simulation_FT[0].Fx;
    sharedSEN->FT[0].Fy = sharedREF->Simulation_FT[0].Fy;
    sharedSEN->FT[0].Fz = sharedREF->Simulation_FT[0].Fz;
    sharedSEN->FT[1].Mx = sharedREF->Simulation_FT[1].Mx;
    sharedSEN->FT[1].My = sharedREF->Simulation_FT[1].My;
    sharedSEN->FT[1].Mz = sharedREF->Simulation_FT[1].Mz;
    sharedSEN->FT[1].Fx = sharedREF->Simulation_FT[1].Fx;
    sharedSEN->FT[1].Fy = sharedREF->Simulation_FT[1].Fy;
    sharedSEN->FT[1].Fz = sharedREF->Simulation_FT[1].Fz;

}


//==============================================================================
//==============================================================================
//==============================================================================

void WriteValveReference(void)
{
    for(int i=0; i<_NO_OF_VC; i++){

        for(int j=0; j<MAX_JOINT; j++){
            int motionOwner = sharedCMD->MotionOwner[i][j];

            sharedSEN->ENCODER[i][j].CurrentRefAngle = sharedREF->AngleReference[motionOwner][i][j];
            sharedSEN->ENCODER[i][j].CurrentRefAngVel = sharedREF->AngVelReference[motionOwner][i][j];
            sharedSEN->ENCODER[i][j].CurrentRefTorque = sharedREF->TorqueReference[motionOwner][i][j];

            finalJointPosRef[j] = sharedREF->ActPosReference[motionOwner][i][j];
            _DEV_VC[i].Joints[j].HCB_Ref.ReferencePosition = finalJointPosRef[j]; // Actuator Position (Unit : mm or deg)
            sharedSEN->ENCODER[i][j].CurrentRefActPos = finalJointPosRef[j];

            finalJointVelRef[j] = sharedREF->ActVelReference[motionOwner][i][j];
            _DEV_VC[i].Joints[j].HCB_Ref.ReferenceVelocity = finalJointVelRef[j]; // Actuator Velocity (Unit : mm/s or deg/s)
            sharedSEN->ENCODER[i][j].CurrentRefActVel = finalJointVelRef[j];

            finalJointFTRef[j] = sharedREF->ActForceReference[motionOwner][i][j];
            _DEV_VC[i].Joints[j].HCB_Ref.ReferenceForceTorque = finalJointFTRef[j]; // Actuator Force or Torque (Unit : N or Nm)
            sharedSEN->ENCODER[i][j].CurrentRefActForce = finalJointFTRef[j];

            finalValvePosRef[j] = sharedREF->ValvePosReference[motionOwner][i][j];
            if(_DEV_VC[i].VALVE_TYPE == "DDV") {
                if(finalValvePosRef[j]>4096.0) finalValvePosRef[j] = 4096.0;
                else if (finalValvePosRef[j]<0.0) finalValvePosRef[j] = 0.0;
            }
            _DEV_VC[i].Joints[j].HCB_Ref.ReferenceValvePos = finalValvePosRef[j]; // Valve Position (Unit : -10000~10000pulse)
            sharedSEN->ENCODER[i][j].CurrentRefValvePos = finalValvePosRef[j];

            finalPWMRef[j] = sharedREF->PWMReference[motionOwner][i][j];
            _DEV_VC[i].Joints[j].HCB_Ref.ReferencePWM = finalPWMRef[j]; // Valve Input Voltage (Unit : -12000~12000mV)
            sharedSEN->ENCODER[i][j].CurrentRefPWM = finalPWMRef[j];

            _DEV_VC[i].Joints[j].HCB_Ref.ReferencePumpPressure = sharedREF->PumpPressureReference[0];

            // Send Reference Data to Board
            switch(sharedREF->ValveCtrlMode_Command[i]) {
            case ValveControlMode_Null: // Valve Control Mode : Null
            {
                cout << "\033[1;34m Valve Control Mode (Board ID : " << _DEV_VC[i].BOARD_ID << ") : Null \033[0m" << endl;
                if(_DEV_VC[i].ConnectionStatus) _DEV_VC[i].CMD_ControlMode(0);
                sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_Noact;
                sharedREF->ValveCtrlMode[i] = ValveControlMode_Null;
                break;
            }
            case ValveControlMode_PosOrFor: // Valve Control Mode : Position or Force Control
            {
                cout << "\033[1;34m Valve Control Mode (Board ID : " << _DEV_VC[i].BOARD_ID << ") : Position or Force \033[0m" << endl;
                if(_DEV_VC[i].ConnectionStatus)_DEV_VC[i].CMD_ControlMode(1);
                sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_Noact;
                sharedREF->ValveCtrlMode[i] = ValveControlMode_PosOrFor;
                break;
            }
            case ValveControlMode_Opening: // Valve Control Mode : Valve Opening
            {
                cout << "\033[1;34m Valve Control Mode (Board ID : " << _DEV_VC[i].BOARD_ID << ") : Valve Position \033[0m" << endl;
                if(_DEV_VC[i].ConnectionStatus)_DEV_VC[i].CMD_ControlMode(2);
                sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_Noact;
                sharedREF->ValveCtrlMode[i] = ValveControlMode_Opening;
                break;
            }
            case ValveControlMode_PWM: // Valve Control Mode : Valve Voltage
            {
                cout << "\033[1;34m Valve Control Mode (Board ID : " << _DEV_VC[i].BOARD_ID << ") : Voltage (PWM) \033[0m" << endl;
                if(_DEV_VC[i].ConnectionStatus)_DEV_VC[i].CMD_ControlMode(3);
                sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_Noact;
                sharedREF->ValveCtrlMode[i] = ValveControlMode_PWM;
                break;
            }
            default:
                break;
            }

            switch(sharedREF->ValveCtrlMode[i]) {
            case ValveControlMode_Null: // Valve Control Mode : Null
            {
                break;
            }
            case ValveControlMode_PosOrFor: // Valve Control Mode : Position or Force Control
            {
                // Joint Stiffness and Damping Change
                if(sharedREF->StiffnDampChange[i] == true) {
                    double argDouble[4] = {0,};
                    argDouble[0] = 3;
                    argDouble[1] = sharedREF->ActuatorStiffness[i];
                    argDouble[2] = sharedREF->ActuatorDamping[i];
                    if(_DEV_VC[i].ConnectionStatus) _DEV_VC[i].SendGeneralMSG(ValveController_GeneralMSG_CMD_PIDGAIN, argDouble);
                    sharedREF->StiffnDampChange[i] = false;
                }

                // Control Method Changed
                if(sharedREF->PosOrFor_Selection[i] != sharedREF->PosOrFor_Selection_last[i]) {
                    if(sharedREF->PosOrFor_Selection[i] == true) {
                        argInt[0] = 1; // Force Control
                        if(_DEV_VC[i].ConnectionStatus) _DEV_VC[i].SendGeneralMSG(ValveController_GeneralMSG_CMD_MODETRANSITION,argInt);
                    } else if (sharedREF->PosOrFor_Selection[i] == false) {
                        argInt[0] = 3; // Position Control
                        if(_DEV_VC[i].ConnectionStatus) _DEV_VC[i].SendGeneralMSG(ValveController_GeneralMSG_CMD_MODETRANSITION,argInt);
                    }
                }
                sharedREF->PosOrFor_Selection_last[i] = sharedREF->PosOrFor_Selection[i];

                //                    bool SupplyPressureChangeOnOff = sharedREF->PumpSupplyPressureChange;
                //                    _DEV_VC[i].SendReference_PosVel(SupplyPressureChangeOnOff);
                break;
            }
            case ValveControlMode_Opening: // Valve Control Mode : Valve Opening
            {
                break;
            }
            case ValveControlMode_PWM: // Valve Control Mode : Valve Voltage
            {
                break;
            }
            case ValveControlMode_UtilMode: // Valve Control Mode : Find Home
            {

                break;
            }
            default:
                break;
            }
        }
    }
}

void WritePumpReference(void)
{
    for(int i=0;i<_NO_OF_PC; i++)
    {
        finalPumpVelocityRef = sharedREF->PumpVelocityReference[i];
        _DEV_PC[i].ReferencePumpVelocity = finalPumpVelocityRef;
        sharedSEN->PUMP[i].CurrentRefVelocity = finalPumpVelocityRef;

        sharedSEN->PUMP[i].CurrentRefPressure = sharedREF->PumpPressureReference[i];

        // Send Reference Data to Board ================================================================
        switch(sharedREF->PumpCtrlMode_Command[i]) {
        case PumpControlMode_Null:
        {
            cout << "\033[1;34m Pump Control Mode (Board ID : " << _DEV_PC[i].BOARD_ID << ") : Null \033[0m" << endl;
            sharedREF->PumpCtrlMode_Command[i] = PumpControlMode_Noact;
            sharedREF->PumpCtrlMode[i] = PumpControlMode_Null;
            break;
        }
        case PumpControlMode_Interpolation:
        {
            cout << "\033[1;34m Pump Control Mode (Board ID : " << _DEV_PC[i].BOARD_ID << ") : Interpolation mode \033[0m" << endl;
            sharedREF->PumpCtrlMode_Command[i] = PumpControlMode_Noact;
            sharedREF->PumpCtrlMode[i] = PumpControlMode_Interpolation;
            break;
        }
        case PumpControlMode_ActiveControl:
        {
            cout << "\033[1;34m Pump Control Mode (Board ID : " << _DEV_PC[i].BOARD_ID << ") : Active Control mode \033[0m" << endl;
            sharedREF->PumpCtrlMode_Command[i] = PumpControlMode_Noact;
            sharedREF->PumpCtrlMode[i] = PumpControlMode_ActiveControl;
            break;
        }
        default:
            break;
        }

        switch(sharedREF->PumpCtrlMode[i]) {
        case PumpControlMode_Null:
        {
            break;
        }
        case PumpControlMode_Interpolation:
        {
            break;
        }
        case PumpControlMode_ActiveControl:
        {
            break;
        }
        default:
            break;
        }
    }
}
