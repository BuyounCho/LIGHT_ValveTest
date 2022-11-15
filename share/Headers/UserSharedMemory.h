#ifndef USERSHAREDMEMORY_H
#define USERSHAREDMEMORY_H

#define USER_SHM_NAME         "USER_SHARED_MEMORY"

#ifndef __LAN_STRUCT_GENERAL_COMMAND_DEF__
#define __LAN_STRUCT_GENERAL_COMMAND_DEF__

typedef struct __LAN_STRUCT_GENERAL_COMMAND_
{
    char    param_c[10];
    int     param_i[10];
    float   param_f[10];
    int     cmd;
} LAN_GENERAL_COMMAND, *pLAN_GENERAL_COMMAND;

#endif

typedef struct _MOTION2GUI_
{
    float   curFootR[6];
    float   curFootL[6];
    float   curZMP[3];
    float   curPEL[3];
    float   _INIT_PEL[3];
    float   _INIT_COM[3];
    float   _ADDCOM;
    float   _qPEL[4];

    float   DRILL_Data[10];

    double  pRF[3];
    double  pLF[3];
    double  pRH[3];
    double  pLH[3];
    double  qRF[4];
    double  qLF[4];
    double  qRH[4];
    double  qLH[4];
    double  qPel[4];
    double  Relb;
    double  Lelb;
    double  rWST;
    double  pCOM[3];
    double  pPelZ;

    int     valveMode;
    //Car Descending
    float ROI_max_X;
    float ROI_min_X;
    float ROI_max_Y;
    float ROI_min_Y;
    float ROI_max_Z;
    float ROI_min_Z;

    float           obj_pos[3];


    ////////////////////////QUAD

    double          RFFT, LFFT, RHFT, LHFT;
    double          Q[4];
    double          Angle[3];
    double          Omega[3];
} MOTION2GUI, *pMOTION2GUI;

typedef struct _GUI2MOTION_
{
//    double  walkingDSP[400];
//    int StepNum;
//    double StepLength;
//    double StepAngle;
//    double StepTime;
//    int WalkingModeCommand;
//    int WalkingStopModeCommand;
//    int WalkingGoalGoModeCommand;

    // LIGHT Walking Commands with Joystick
    int JoyStick_OnOff;

    char JoyStick_BTN[12];
    double JoyStick_WalkingCommand_X;
    double JoyStick_WalkingCommand_Y;
    double JoyStick_WalkingCommand_Yaw;
    int JoyStick_WalkingCommand_Start;
    int JoyStick_WalkingCommand_Stop;

//    bool JoyStick_CommandSet[10];

//    LAN_GENERAL_COMMAND ros_cmd;
} GUI2MOTION, *pGUI2MOTION;

typedef struct _USER_SHM_
{
    MOTION2GUI  M2G;
    GUI2MOTION  G2M;

    double			WalkReadyCOM[3];

} USER_SHM, *pUSER_SHM;




#endif // USERSHAREDMEMORY_H
