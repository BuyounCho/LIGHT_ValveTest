#ifndef RB_SHARED_MEMORY_H
#define RB_SHARED_MEMORY_H

#define RBCORE_SHM_NAME_REFERENCE       "RBCORE_SHARED_MEMORY_REFERENCE"
#define RBCORE_SHM_NAME_SENSOR          "RBCORE_SHARED_MEMORY_SENSOR"
#define RBCORE_SHM_NAME_COMMAND         "RBCORE_SHARED_MEMORY_COMMAND"

#define MOTOR_1CH               1
#define MOTOR_2CH               2

#define MAX_VC      13
#define MAX_JOINT   MOTOR_1CH
#define MAX_FT      2
#define MAX_IMU     1
#define MAX_SP      1
#define MAX_OF      2
#define MAX_AL      10
#define MAX_PC      1

#define MAX_PREVIEW             25

#define MAX_HCB_CAN_CHANNEL         7

#define MAX_MANUAL_CAN		30
#define MAX_COMMAND_DATA        50

#define COMMAND_CANID           0x01
#define SENSOR_REQUEST_CANID    0x03    //0x02

#define RT_TIMER_PERIOD_MS      2.5
#define RT_TIMER_PERIOD_US      2500
#define RT_TIMER_FREQ           400.0f

#define SYS_DT_CAN_US           2500
#define SYS_FREQ_CAN            400.0f

//#define SYS_DT_PUMPING          0.005f
//#define SYS_FREQ_PUMPING        200.0f

//#define SYS_DT_WALKING          0.005f
//#define SYS_FREQ_WALKING        200.0f

#define SYS_DT_ACTTEST          0.0025f
#define SYS_FREQ_ACTTEST        400.0f

#define SYS_DT_VALVETEST          0.0025f
#define SYS_FREQ_VALVETEST        400.0f

#define EXTERNAL
//#define LOCAL
//#define SIMULATION

#define RBCORE_PODO_NO          0

#define ENABLE			1
#define DISABLE			0

#define     PI         3.141592f
#define     R2D        57.2957802f  //(180.0f/3.141592f)
#define     D2R        0.0174533f   //(3.141592f/180.0f)
#define     g_const    9.81f

#include <stdint.h>
#include <time.h>

typedef enum{
    MANUALCAN_NEW = 0,
    MANUALCAN_EMPTY,
    MANUALCAN_WRITING
} MANUALCAN_STATUS;

typedef	struct _MANUAL_CAN_{
    unsigned char	channel;
    unsigned int        id;
    unsigned char	data[8];
    unsigned char	dlc;
    MANUALCAN_STATUS    status;
} MANUAL_CAN;

typedef union{
    struct{
        unsigned    HIP:1;	 	// Open loop(PWM) Control mode
        unsigned    RUN:1;		// Control ON
        unsigned    MOD:1;		// Control mode
        unsigned    LIM:1;		// Limit sensor
        unsigned    HME:4;		// Limit Sensor mode during Homing

        unsigned    JAM:1;		// JAM error
        unsigned    PWM:1;		// PWM saturation
        unsigned    BIG:1;		// Big Position error
        unsigned    INP:1;              // Big Input error
        unsigned    FLT:1;		// FET Driver Fault 1= Fault
        unsigned    ENC:1;              // encoder fail
        unsigned    CUR:1;		// big current difference <<== used for CAN connection check
        unsigned    TMP:1;		// Temperature warning

        unsigned    PS1:1;		// Position Limit 1
        unsigned    PS2:1;		// Position Limit 2
        unsigned    SWM:1;		// switching mode (1:complementary, 0:non-complementary)
        unsigned    GOV:1;		// gain override
        unsigned    FRC:1;		// friction compensation
        unsigned    REF:1;		// reference mode (1:incremental, 0:absolute)
        unsigned    CAN:1;		// CAN has been recovered

        unsigned    RPL:1;              // reply ok
    }b;
    unsigned char B[3];
}mSTAT;

struct COCOA_DATA
{
    double          P_KP, P_KI, P_KD;
    double          C_KP, C_KI, C_KD;
    double          FOC_P_KP, FOC_P_KI, FOC_P_KD;
    double          FOC_C_KP, FOC_C_KI, FOC_C_KD;
    double          PWM_RATE_LIMIT;
    double          CURRENT_LIMIT;
    uint16_t        FINDHOME_SEARCH_VEL;
    uint16_t        FINDHOME_OFFSET;
    bool            FINDHOME_DIRECTION;//true=>positive
    bool            FINDHOME_OFFSET_DIRECTION;//true=>positive
    bool            MOTOR_DIRECTION;//true=>positive
    bool            FET_ONOFF;
    int             BOARD_ACT;
    int             TORQUE_SENSOR1;
    int             TORQUE_SENSOR2;

    // Newly added by HSW
    double          EMF_KP;
    uint16_t        VEL_LIMIT;
    uint16_t        ACC_LIMIT;
    uint16_t        GAIN_OVER_VALUE;
    int             FINDHOME_LIM;
    uint16_t        ERROR_LIM;
    bool            BIGERROR_ONOFF;
    bool            CANERROR_ONOFF;
    bool            ENCERROR_ONOFF;
    bool            HOMEERROR_ONOFF;
    bool            PLIMITERROR_ONOFF;
    bool            LOGICERROR_ONOFF;
    bool            INPUTERROR_ONOFF;

    int             HOME_STATE;
    double          BEMF;

};

struct HCB_REF
{
    // Revised for Hydraulic Actuator (ActuatorLevel)
    double          ReferencePosition;      // LinearAct : mm, RotaryAct : deg
    double          ReferenceVelocity;      // LinearAct : mm/s, RotaryAct : deg/s
    double          ReferenceForceTorque;   // LinearAct : N, RotaryAct : Nm
    double          ReferencePumpPressure;  // Pump Supply Pressure
    double          ReferenceValvePos;      // DDV : pulse, two-stage : uA
    double          ReferencePWM;           // Valve Input Voltage
    HCB_REF();
};

inline HCB_REF::HCB_REF(){

}

struct HCB_DATA
{
    // Revised for Hydraulic Actuator (ActuatorLevel)
    float           CurrentPosition; // LinearAct : mm, RotaryAct : deg
    float           CurrentVelocity; // LinearAct : mm/s, RotaryAct : deg/s
    float           CurrentForce;    // LinearAct : N, RotaryAct : Nm
    float           CurrentValvePos; // DDV : pulse, two-stage : N/A
    float           CurrentValvePosRef; // (Calculated in Valve Position) DDV : pulse, two-stage : N/A
    float           CurrentPWM;      // Voltage : V
    float           CurrentPressureA; // LinearAct : bar, RotaryAct : bar
    float           CurrentPressureB; // LinearAct : bar, RotaryAct : bar
    int16_t         CurrentTempData1; // -32756~32756
    int16_t         CurrentTempData2; // -32756~32756
    int16_t         CurrentTempData3; // -32756~32756
    int16_t         CurrentTempData4; // -32756~32756
};

struct HCB_INFO
{
    int             CAN_FREQ;
    bool            FET_ONOFF;
    bool            REQUEST_ONOFF_PosVel;
    bool            REQUEST_ONOFF_ValvePosnPWM;
    bool            REQUEST_ONOFF_Pressure;
    bool            REQUEST_ONOFF_OtherInfo;

    bool            BIGERROR_ONOFF;
    bool            ENCERROR_ONOFF;
    bool            CANERROR_ONOFF;
    bool            HOMEERROR_ONOFF;
    bool            PLIMITERROR_ONOFF;
    bool            LOGICERROR_ONOFF;
    bool            INPUTERROR_ONOFF;
    int             CONTROL_MODE;
    int             OPERATION_MODE;  // (00 : Moog & Rot, 01 : Moog & Lin, 10 : KNR & Rot, 11 : KNR & Lin)
    int             SENSING_MODE;    // (0 : torque, 1: pressure)
    int             CURRENT_CONTROL_MODE;   // (0 : pwm, 1 : current control)
    int             JOINTENC_DIRECTION;     // +1:Positive, -1:Negative
    int             VALVEINPUT_DIRECTION;   // +1:Positive, -1:Negative
    int             VALVEENC_DIRECTION;     // +1:Positive, -1:Negative
    double          BOARD_IN_VOLTAGE;
    double          BOARD_OPER_VOLTAGE;
    int             VARIABLE_SUPPLYPRES_ONOFF;    // (0 : OFF, 1: ON)

    int             VALVE_CENTER_POS;
    int             VALVE_DZ_PLUS, VALVE_DZ_MINUS;
    int             VEL_COMPENSATION_K;
    int             ACTUATOR_COMPLIANCE_K;
    int             VALVE_FEEDFORWARD;
    int             BULK_MODULUS;
    int             VOL_A, VOL_B;
    int             PIS_AREA_A, PIS_AREA_B;
    int             SUP_PRES, RET_PRES;
    int             JOINTENC_LIMIT_MINUS, JOINTENC_LIMIT_PLUS;
    int             PIS_STROKE;
    int             VALVEPOS_LIMIT_MINUS, VALVEPOS_LIMIT_PLUS;
    double          JOINTENC_PPP;
    double          FORCESEN_PPF;
    double          PRESSEN_PPP_A;
    double          PRESSEN_PPP_B;
    int             CONST_FRIC;
    int             HOMEPOS_OFFSET;
    int             HOMEPOS_VALVE_OPENING;
    double          VALVE_GAIN_PLUS[5];
    double          VALVE_GAIN_MINUS[5];
    double          VALVE_PWM_VALVE_ID[41];
    double          VALVE_POS_VALVE_ID[41];
    double          VALVE_POSITION_FLOWRATE_ID[51];
    double          VALVE_FLOWRATE_FLOWRATE_ID[51];

    double          VALVE_P_KP, VALVE_P_KI, VALVE_P_KD;
    double          JOINT_P_KP, JOINT_P_KI, JOINT_P_KD;
    double          JOINT_F_KP, JOINT_F_KI, JOINT_F_KD;
    double          JOINT_SPRING, JOINT_DAMPER;

    HCB_INFO(){
        PRESSEN_PPP_A = 15.00;
        PRESSEN_PPP_B = 15.00;
        FORCESEN_PPF = 0.2773;
    }
};

typedef struct _ENCODER_SENSOR_
{
    int     BoardConnection;

    // Revised for Hydraulic Actuator (JointLevel)
    float      CurrentRefAngle; // Angle reference
    float      CurrentRefAngVel; // Angular velocity reference
    float      CurrentRefActPos; // Cylinder linear position reference
    float      CurrentRefActVel; // Cylinder linear velocity reference
    float      CurrentRefTorque; // Angle reference
    float      CurrentRefActForce; // Cylinder linear force reference
    float      CurrentRefValvePos; // Valve pos reference
    float      CurrentRefPWM; // Valve Input Voltage reference

    float      CurrentAngle; // deg
    float      CurrentAngVel; // deg/s
    float      CurrentActPos; // mm or deg
    float      CurrentActVel; // mm/s or deg/s
    float      CurrentActForce; // N or Nm
    float      CurrentTorque; // Nm
    int        CurrentValvePos; // DDV : pulse, two-stage : uA
    int        CurrentValvePosRef; // (Calculated in Valve Position) DDV : pulse, two-stage : uA
    int        CurrentPWM;
    float      CurrentPressureA; // bar
    float      CurrentPressureB; // bar
    int16_t    CurrentData1;
    int16_t    CurrentData2;
    int16_t    CurrentData3;
    int16_t    CurrentData4;

    int         NO_RESPONSE_CNT;

    HCB_INFO    HCB_Info;

}ENCODER_SENSOR;

typedef struct _PUMP_SENSOR_
{
    int         BoardConnection;
    double      CurrentSettingVoltage; // Setting Voltage [V]

    // Revised for Hydraulic Actuator (JointLevel)
    double      CurrentRefPressure; // Pressure reference [bar]
    double      CurrentRefVelocity; // Velocity reference [rpm]

    double      CurrentPressure;    // Sensing pressure [bar]
    double      CurrentVelocity;    // Sensing velocity [rpm]
    double      CurrentTemperature; // Sensing stator temperature [deg]

    double      CurrentMechaPower;   // Mechanical Power [W]
    double      CurrentHydraPower;   // Hydraulic Power [W]

    int         NO_RESPONSE_CNT;
}PUMP_SENSOR;

typedef struct _FT_SENSOR_
{
    int     BoardConnection;
    float   Mx;
    float   My;
    float   Mz;
    float   Fx;
    float   Fy;
    float   Fz;
    float   RollVel;
    float   PitchVel;
    float   Roll;
    float   Pitch;
}FT_SENSOR;

typedef struct _IMU_SENSOR_
{
    int     BoardConnection;
    float   Roll;  // (Global >> IMU) X''-axis rotation
    float   Pitch;
    float   Yaw;
    float   Wx_B;  // IMU(Body)'s angular velocity w.r.t the body frame
    float   Wy_B;
    float   Wz_B;
    float   Ax_B;  // IMU's linear acceleration w.r.t the body frame
    float   Ay_B;
    float   Az_B;
    float   Wx_G;  // IMU's angular velocity w.r.t the global frame
    float   Wy_G;
    float   Wz_G;
    float   Ax_G;  // IMU's linear acceleration w.r.t the global frame
    float   Ay_G;
    float   Az_G;
    double  Q[4]; // Q[0]:w , Q[1~3]:x,y,z
}IMU_SENSOR;

typedef struct _OF_SENSOR_
{
    int     BoardConnection;
    int     AccumX[2];
    int     AccumY[2];
    int     DeltaX[2];
    int     DeltaY[2];
    float   Xmea;
    float   Ymea;
    float   thmea;
    float   Pitmea;
}OF_SENSOR;

typedef struct _SMART_POWER_
{
    int     BoardConnection;

    float   Voltage;
    float   Current;
}SMART_POWER;

typedef struct _FOG_SENSOR_
{
    float   Roll;
    float   Pitch;
    float   Yaw;
    float   RollVel;
    float   PitchVel;
    float   YawVel;
    int     Warning;
    double  quaternion[4];
    double  LocalW[3];
}FOG_SENSOR;

typedef struct _COMMAND_STRUCT_
{
    int     USER_COMMAND;
    char    JOYSTICK_COMMAND_ONOFF;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA];
    int	    USER_PARA_INT[MAX_COMMAND_DATA];
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA];
    double  USER_PARA_DOUBLE[MAX_COMMAND_DATA];

    HCB_INFO    HCB_Info;

} COMMAND_STRUCT, *pCOMMAND_STRUCT;

typedef struct _USER_COMMAND_
{
    int             COMMAND_TARGET;
    COMMAND_STRUCT  COMMAND_DATA;
} USER_COMMAND, *pUSER_COMMAND;

//======================================
// Sensor Shared Memory <Read Only>
typedef struct _RBCORE_SHM_SENSOR_
{
    char            PODO_AL_WORKING[MAX_AL];
    mSTAT           MCStatus[MAX_VC][MOTOR_2CH];
    float           MCTemperature[MAX_VC];
    float           MotorTemperature[MAX_VC][MOTOR_2CH];

    ENCODER_SENSOR  ENCODER[MAX_VC][MOTOR_2CH];
    PUMP_SENSOR     PUMP[MAX_PC];

    FT_SENSOR       FT[MAX_FT];
    IMU_SENSOR      IMU[MAX_IMU];
    FOG_SENSOR      FOG;
    OF_SENSOR       OF;
    SMART_POWER     SP[MAX_SP];

    int             CAN_Enabled;
    int             REF_PosVel_Enabled;
    int             REF_Force_Enabled;
    int             REF_LoadPres_Enabled;
    int             REF_PWM_Enabled;
    int             REF_ValvePos_Enabled;

    int             SEN_Enabled;
    int             ENC_Enabled;

    int             Sim_Time_sec;
    int             Sim_Time_nsec;

    int             TORQUE_SENSOR1;
    int             TORQUE_SENSOR2;

    int             CONTACT_SENSOR_POS[4];
    int             CONTACT_SENSOR_VEL[4];

}RBCORE_SHM_SENSOR, *pRBCORE_SHM_SENSOR;
//======================================

//======================================
// Reference Shared Memory <Write Only>
typedef struct _RBCORE_SHM_REFERENCE_
{
    // This Variables are for choreonoid simulation. (20190508, Buyoun) ////
    double          Simulation_AngleSensor[MAX_VC];  // sensor value from choreonoid
    double          Simulation_VelocitySensor[MAX_VC];  // sensor value from choreonoid
    double          Simulation_TorqueSensor[MAX_VC]; // sensor value from choreonoid
    FT_SENSOR       Simulation_FT[MAX_FT];
    IMU_SENSOR      Simulation_IMU[MAX_IMU];
    int             Simulation_HandlingMode;
    bool            Simulation_DataEnable;
    // /////////////////////////////////////////////////////////////////////

    double          AngleReference[MAX_AL][MAX_VC][MOTOR_1CH];     // Joint Angle, not Cylinder Position! (by BUYOUN)
    double          AngVelReference[MAX_AL][MAX_VC][MOTOR_1CH];    // Joint Angular Vel, not Cylinder Velocity! (by BUYOUN)
    double          TorqueReference[MAX_AL][MAX_VC][MOTOR_1CH];    // Joint Torque, not Cylinder Force! (by BUYOUN)
    double          ActPosReference[MAX_AL][MAX_VC][MOTOR_1CH];
    double          ActVelReference[MAX_AL][MAX_VC][MOTOR_1CH];
    double          ActForceReference[MAX_AL][MAX_VC][MOTOR_1CH];
    double          JointStiffness[MAX_VC];
    double          JointDamping[MAX_VC];
    double          ActuatorStiffness[MAX_VC];
    double          ActuatorDamping[MAX_VC];
    bool            StiffnDampChange[MAX_VC];

    double          ValvePosReference[MAX_AL][MAX_VC][MOTOR_1CH];
    double          PWMReference[MAX_AL][MAX_VC][MOTOR_1CH];

    int             ValveCtrlMode_Command[MAX_VC]; // 0 : Nothing, 1 : Position or Force, 2 : Valve Open (Openloop)
    int             ValveCtrlMode[MAX_VC];
    bool            PosOrFor_Selection[MAX_VC];  // 0(false):PositionControl, 1(true):TorqueControl
    bool            PosOrFor_Selection_last[MAX_VC];

    int             PumpCtrlMode_Command[MAX_PC]; // 0 : Nothing, 1 : Interpolation Mode, 2 : Instantaneous Mode (Active Control)
    int             PumpCtrlMode[MAX_PC];
    double          PumpVelocityReference[MAX_PC];
    double          PumpPressureReference[MAX_PC];
    bool            PumpSupplyPressureChange;

    // Future Reference for Model Predictive Control (Pump Control)
    int             N_PrevPump = 26;
    double          dT_PrevPump = 0.060;
    bool            Flag_PumpControlMPC;
    double          RequiredFlowrateReference_Future[MAX_PREVIEW+1]; // Total Flowrate, 0:Current Ref, 1~MAX_PREVIEW:Future Ref
    double          RequiredPressureReference_Future[MAX_PREVIEW+1]; // Total Supply Pressure, 0:Current Ref, 1~MAX_PREVIEW:Future Ref
    double          ActFlowrateReference_Future[MAX_PREVIEW+1][MAX_VC]; // Flowrate at each actuator, 0:Current Ref, 1~MAX_PREVIEW:Future Ref
    double          LoadPressureReference_Future[MAX_PREVIEW+1][MAX_VC]; // Load Pressure at each actuator, 0:Current Ref, 1~MAX_PREVIEW:Future Ref

    int             RESPONSE_CNT_DAEMON;
    int             RESPONSE_CNT_AL;
}RBCORE_SHM_REFERENCE, *pRBCORE_SHM_REFERENCE;
//======================================

//======================================
// Reference Shared Memory <Read Write>
typedef struct _RBCORE_SHM_COMMAND_
{
    bool            ALTHREAD_ONOFF_SIGNAL[MAX_AL];
    int             SYNC_SIGNAL[MAX_AL];
    int             ACK_SIGNAL[MAX_VC][MOTOR_2CH];

    int             MotionOwner[MAX_VC][MOTOR_2CH];
    COMMAND_STRUCT  COMMAND[MAX_AL];
    int             CommandAccept[MAX_AL];
    long            ErrorInform2GUI;

    MANUAL_CAN      ManualCAN[MAX_MANUAL_CAN];
} RBCORE_SHM_COMMAND, *pRBCORE_SHM_COMMAND;
//======================================

typedef enum _DAEMON4LIGHT_COMMAND_SET_{
    DAEMON4LIGHT_NO_ACT = 0,
    // For process handle
    DAEMON4LIGHT_PROCESS_CREATE,
    DAEMON4LIGHT_PROCESS_KILL,

    // For CAN Check
    DAEMON4LIGHT_CAN_CHECK,
//    DAEMON4LIGHT_CAN_CHANNEL_ARRANGE,

    // For LIGHT utility
    DAEMON4LIGHT_SAVEDATA,

    // For HCB(motion controller)
    DAEMON4LIGHT_MOTION_ERRORCLEAR,
    DAEMON4LIGHT_MOTION_ENCODER_ZERO,
    DAEMON4LIGHT_MOTION_FET_ONOFF,
    DAEMON4LIGHT_MOTION_REF_ONOFF,
    DAEMON4LIGHT_MOTION_ERROR_CLEAR,
    DAEMON4LIGHT_MOTION_BOARDTEST,
    DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING,
    DAEMON4LIGHT_MOTION_REF_RESET,
    DAEMON4LIGHT_ENC_ZERO,
    DAEMON4LIGHT_FINDHOME,
    DAEMON4LIGHT_MOTION_REQUEST_ONOFF,
    DAEMON4LIGHT_MOTION_ASK_PARAMETERS,
    DAEMON4LIGHT_MOTION_SET_PARAMETERS,
    DAEMON4LIGHT_MOTION_READnSAVE_PARAMETERS,
    DAEMON4LIGHT_MOTION_LOADnSET_PARAMETERS,
    DAEMON4LIGHT_MOTION_SET_EVERYTHING,
    DAEMON4LIGHT_MOTION_SET_BNO,
    DAEMON4LIGHT_MOTION_CHANGE_POSorTOR,
    DAEMON4LIGHT_MOTION_CHANGE_SUPPLYPRES_MODE,

    // For Valve Performance Test
    DAEMON4LIGHT_VALVEPERFTEST_REQUEST_ONOFF,
    DAEMON4LIGHT_VALVEPERFTEST_VOLTAGE2VALVEPOS_ID,
    DAEMON4LIGHT_VALVEPERFTEST_VOLTAGE2VALVEPOS_ASKRESULT,
    DAEMON4LIGHT_VALVEPERFTEST_DEADZONE_ID,
    DAEMON4LIGHT_VALVEPERFTEST_DEADZONE_ASKRESULT,
    DAEMON4LIGHT_VALVEPERFTEST_VALVEPOS2FLOWRATE_ID,
    DAEMON4LIGHT_VALVEPERFTEST_VALVEPOS2FLOWRATE_ASKRESULT,

    // For Pump Control
    DAEMON4LIGHT_PUMP_ASK_STATUS,
    DAEMON4LIGHT_PUMP_SEND_COMMAND,
    DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET,

    // For FT Sensor
    DAEMON4LIGHT_SENSOR_FT_ONOFF,
    DAEMON4LIGHT_SENSOR_FT_NULL,

    // For IMU
    DAEMON4LIGHT_SENSOR_IMU_ONOFF,
    DAEMON4LIGHT_SENSOR_IMU_NULL,
    DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET,

    // For Simulation Setting
    DAEMON4LIGHT_SIMLATION_MODE_ONOFF,

} COMMAND_SET;

typedef enum LIGHT_WALKING_COMMAND_SET {
    LIGHT_NO_ACT = 100,
    LIGHT_ALL_STOP,
    LIGHT_DATA_SAVE,
    LIGHT_DATA_SAVE_SUPPRES,
    LIGHT_DATA_SAVE_SYS_ID,

    LIGHT_PARAMETER_SETTING,
    LIGHT_SUPPLYPRESSURE_SETTING,

    LIGHT_ANKLETORQUECOMP_ONOFF,

    // Joint space control
    LIGHT_GOTO_HOMEPOSE,

    // Task space control - Base is floating
    LIGHT_WORKSPACE_MOVE,
    LIGHT_AIRWALKING,

    // Task space control - Contact to ground
    LIGHT_COM_MOVING_DSP,
    LIGHT_COM_MOVING_SSP,
    LIGHT_SUPPORT_TRANSITION,
    LIGHT_SYSID_COM,

    LIGHT_SQUAT,

    LIGHT_COM_MOVING_RDSP_SMOOTH,
    LIGHT_COM_MOVING_LDSP_SMOOTH,

    LIGHT_RFSWINGUP_DYNAMIC,
    LIGHT_RFSWINGDOWN_DYNAMIC,
    LIGHT_LFSWINGUP_DYNAMIC,
    LIGHT_LFSWINGDOWN_DYNAMIC,

    LIGHT_WALK,
    LIGHT_WALK_withJOYSTICK,

    LIGHT_JUMPTEST,

    LIGHT_FULLTASK

}LIGHT_WALKING_COMMAND;

typedef enum PUMP_CONTROL_COMMAND_SET{
    PUMP_CONTROL_NO_ACT = 100,
    PUMP_CONTROL_SAVE_DATA,

    PUMP_CONTROL_PRESREF_SELECTION,

    PUMP_ACTIVE_CONTROL_ONOFF,
    PUMP_ACTIVE_CONTROL_MPC_ONOFF,

    PUMP_ACTIVE_CONTROL_LINEAR,

}PUMP_CONTROL_COMMAND;

enum PumpController_GeneralMSGSET {
    PumpController_GeneralMSG_CANCHECK = 0,

    PumpController_GeneralMSG_ASK_SPEEDREF = 81,
    PumpController_GeneralMSG_ASK_CTRL_ONOFF = 82,
    PumpController_GeneralMSG_ASK_DATAREQUESTFLAG = 90,

    PumpController_GeneralMSG_CMD_SPEEDREF = 181,
    PumpController_GeneralMSG_CMD_CTRL_ON = 182,
    PumpController_GeneralMSG_CMD_CTRL_OFF = 183,
    PumpController_GeneralMSG_CMD_PRESSURENULL = 188,
    PumpController_GeneralMSG_CMD_DATAREQUESTFLAG = 190,
};

typedef enum SH_TASK_COMMAND_SET{

    SH_TASK_NO_ACT = 100,
    SH_TASK_SAVE_DATA,

    SH_TASK_VALVE_ID_POS,
    SH_TASK_VALVE_ID_NEG,

    // For process handle
    SH_TASK_CONST_OPENING,
    SH_TASK_SINEWAVE_OPENING,
    SH_TASK_JUMP_OPENING,

    SH_TASK_POSITION_CONTROL_HOMEPOSE,
    SH_TASK_POSITION_CONTROL_WALKREADYPOSE,
    SH_TASK_POSITION_CONTROL_GOTO_POSE,
    SH_TASK_POSITION_CONTROL_RANDOM_MOTION,
    SH_TASK_POSITION_CONTROL_SINEWAVE,

    SH_TASK_TORQUE_CONTROL_GO,
    SH_TASK_TORQUE_CONTROL_STOP,

}SH_TASK_COMMAND;

typedef enum VALVEPERFTEST_COMMAND_SET{

    VALVEPERFTEST_NO_ACT = 100,
    VALVEPERFTEST_SAVE_DATA,

    VALVEPERFTEST_VALVEVOLTAGE_STEP,
    VALVEPERFTEST_VALVEVOLTAGE_SINEWAVE,

    VALVEPERFTEST_VALVEPOSITION_STEP,
    VALVEPERFTEST_VALVEPOSITION_SINEWAVE,

    VALVEPERFTEST_ACTPOSITION_MOVESMOOTH,
    VALVEPERFTEST_ACTPOSITION_SINEWAVE,
    VALVEPERFTEST_ACTPOSITION_SINEWAVE_MULTI,

    VALVEPERFTEST_ACTFORCE_STEP,
    VALVEPERFTEST_ACTFORCE_GRAVCOMP,
    VALVEPERFTEST_ACTFORCE_SINEWAVE,

}VALVEPERFTEST_COMMAND;

typedef enum QUAD_TASK_COMMAND_SET{
    QUAD_TASK_NO_ACT = 100,
    QUAD_TASK_SAVE_DATA,

    // For process handle
    QUAD_TASK_ARMMOTION,

    QUAD_TASK_HOMEPOSE,
    QUAD_TASK_TESTJOINTS,
    QUAD_TASK_SQUAT,
    QUAD_TASK_SWING,

}QUAD_TASK_COMMAND;


enum ValveControlMode
{
    ValveControlMode_Noact      = -1,
    ValveControlMode_Null       = 0,
    ValveControlMode_PosOrFor   = 1,
    ValveControlMode_Opening    = 2,
    ValveControlMode_PWM        = 3,
    ValveControlMode_UtilMode   = 4
};

enum JointControlMode
{
    JointControlMode_Position = 0,
    JointControlMode_Torque
};

enum PumpControlMode
{
    PumpControlMode_Noact = -1,
    PumpControlMode_Null = 0,
    PumpControlMode_Interpolation,
    PumpControlMode_ActiveControl
};

#endif // RB_SHARED_MEMORY_H

