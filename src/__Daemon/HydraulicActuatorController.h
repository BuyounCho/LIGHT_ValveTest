#ifndef HydraulicActuatorController_H
#define HydraulicActuatorController_H

#include "RBDataType.h"
#include "RBLog.h"
//#include "RBCAN.h"
#include "RBCAN_new.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <sys/stat.h>

#define HCB_CTRL_MODE_NOACT                 0   // NO control mode
#define HCB_CTRL_MODE_VALVE_OPENLOOP        1   // valve open loop control mode
#define HCB_CTRL_MODE_VALVE_POS             2   // valve position control mode
#define HCB_CTRL_MODE_POS_FORCE_PWM         3   // joint position and force control based on PWM
#define HCB_CTRL_MODE_POS_FORCE_VALVE       4   // joint position and force control based on valve position
#define HCB_CTRL_MODE_POS_FORCE_LEARN       5   // joint position and force control based on learing
#define HCB_CTRL_MODE_POS_PRES_PWM          6   // joint position and load pressure control based on PWM
#define HCB_CTRL_MODE_POS_PRES_VALVE        7   // joint position and load pressure control based on valve position
#define HCB_CTRL_MODE_POS_PRES_LEARN        8   // joint position and load pressure control based on learning
#define HCB_CTRL_MODE_PWM_SINE_OPENLOOL     9   // valve sine open loop mode

#define HCB_CTRL_MODE_FORCE_NULL            20  // force sensor nulling
#define HCB_CTRL_MODE_VALVE_NULL            21  // valve nulling and dead zone setting
#define HCB_CTRL_MODE_FINDHOME              22  // find home
#define HCB_CTRL_MODE_FLOWGAIN_TUNE         23  // flow rate gain tuning

#define HCB_OPER_MODE_SW                    0 // Sungwoo operation mode
#define HCB_OPER_MODE_SH                    1 // Seunghoon operation mode

#define HCB_REF_MODE_NOACT                  0
#define HCB_REF_COS                         1
#define HCB_REF_LINE                        2
#define HCB_REF_SIN_WAVE                    3


class ValveController
{
public:
    ValveController();

    // from DB <-
    QString     BOARD_NAME;
    unsigned int     BOARD_ID;
    unsigned int     CAN_CHANNEL;
    QString     ACTUATOR_TYPE;
    QString     VALVE_TYPE;

    double      PULSE_PER_POSITION;
    double      PULSE_PER_FORCETORQUE;
    double      PULSE_PER_PRESSURE;

    unsigned int     ID_SEND_GENERAL;
    unsigned int     ID_SEND_POSVEL;
    unsigned int     ID_SEND_VALVEPOSnPWM;

    unsigned int     ID_RCV_GENERAL;
    unsigned int     ID_RCV_POSVEL;
    unsigned int     ID_RCV_VALVEPOSnPWM;
    unsigned int     ID_RCV_PRESSURE;
    unsigned int     ID_RCV_OTHERINFO;
    unsigned int     ID_RCV_ALART;

    // not from DB----
    int     ConnectionStatus;
    int     ErrorFlag;
    float   BoardTemperature;

    RB_JOINT    Joints[MAX_JOINT];

    void GetDBData(DB_VC db);

    int SendGeneralMSG(int _typeMSG, void *arg = NULL);

    int ResetReference_PosVel(bool PsVar_OnOff);

    // Board Operation Setting ///////////////////////////////////////////////
    int CANCheck(void);
    int CANChannel_Arrange(void);
    int InformationCheck(void);

    int CMD_ErrorClear(void);

    int CMD_Request_PosVel(bool OnOff);
    int CMD_Request_ValvePosnPWM(bool OnOff);
    int CMD_Request_Pressure(bool OnOff);
    int CMD_Request_OtherInfo(bool OnOff);

    void CMD_ControlMode(int mode);

private:

};

enum ValveController_GeneralMSGSET {
    ValveController_GeneralMSG_CANCHECK = 0,

    ValveController_GeneralMSG_ASK_BOARDNUMBER = 1,
    ValveController_GeneralMSG_ASK_BOARDOPERATIONMODE = 2,
    ValveController_GeneralMSG_ASK_CANFREQ = 6,
    ValveController_GeneralMSG_ASK_CTRLMODE = 7,
    ValveController_GeneralMSG_ASK_JOINTENCDIR = 9,
    ValveController_GeneralMSG_ASK_VALVEINPUTDIR = 10,
    ValveController_GeneralMSG_ASK_VALVEENCDIR = 11,
    ValveController_GeneralMSG_ASK_BOARDINPUTVOLTAGE = 12,
    ValveController_GeneralMSG_ASK_VALVEOPERVOLTAGE = 13,
    ValveController_GeneralMSG_ASK_VARIABLESUPPLY_ONOFF = 15,
    ValveController_GeneralMSG_ASK_PIDGAIN = 20,
    ValveController_GeneralMSG_ASK_VALVEDZ = 21,
    ValveController_GeneralMSG_ASK_VELCOMP = 22,
    ValveController_GeneralMSG_ASK_COMPLIANCE = 23,
    ValveController_GeneralMSG_ASK_FEEDFORWARD = 25,
    ValveController_GeneralMSG_ASK_BULKMODULUS = 26,
    ValveController_GeneralMSG_ASK_CHAMBERVOL = 27,
    ValveController_GeneralMSG_ASK_PISTONAREA = 28,
    ValveController_GeneralMSG_ASK_SUPnRETPRES = 29,
    ValveController_GeneralMSG_ASK_JOINTENCLIMIT = 30,
    ValveController_GeneralMSG_ASK_PISTONSTROKE = 31,
    ValveController_GeneralMSG_ASK_VALVEPOSLIMIT = 32,
    ValveController_GeneralMSG_ASK_ENCPPP = 33,
    ValveController_GeneralMSG_ASK_SENPPF = 34,
    ValveController_GeneralMSG_ASK_SENPPP = 35,
    ValveController_GeneralMSG_ASK_CONSTFRICTION = 36,
    ValveController_GeneralMSG_ASK_VALVEGAINPLUS = 37,
    ValveController_GeneralMSG_ASK_VALVEGAINMINUS = 38,
    ValveController_GeneralMSG_ASK_HOMEPOSOFFSET = 40,
    ValveController_GeneralMSG_ASK_HOMEPOSOPENING = 41,
    ValveController_GeneralMSG_ASK_VOLTAGE2VALVEPOS_RESULT = 43,
    ValveController_GeneralMSG_ASK_VALVEPOS2FLOWRATE_RESULT = 44,

    ValveController_GeneralMSG_CMD_BOARDNUMBER = 101,
    ValveController_GeneralMSG_CMD_BOARDOPERATIONMODE = 102,
    ValveController_GeneralMSG_CMD_ENCZERO = 103,
    ValveController_GeneralMSG_CMD_FETONOFF = 104,
    ValveController_GeneralMSG_CMD_MODETRANSITION = 105,
    ValveController_GeneralMSG_CMD_CANFREQ = 106,
    ValveController_GeneralMSG_CMD_CTRLMODE = 107,
    ValveController_GeneralMSG_CMD_DATAREQUEST_ONOFF = 108,
    ValveController_GeneralMSG_CMD_JOINTENCDIR = 109,
    ValveController_GeneralMSG_CMD_VALVEINPUTDIR = 110,
    ValveController_GeneralMSG_CMD_VALVEENCDIR = 111,
    ValveController_GeneralMSG_CMD_BOARDINPUTVOLTAGE = 112,
    ValveController_GeneralMSG_CMD_VALVEOPERVOLTAGE = 113,
    ValveController_GeneralMSG_CMD_VARIABLESUPPLY_ONOFF = 115,
    ValveController_GeneralMSG_CMD_PIDGAIN = 120,
    ValveController_GeneralMSG_CMD_VALVEDZ = 121,
    ValveController_GeneralMSG_CMD_VELCOMP = 122,
    ValveController_GeneralMSG_CMD_COMPLIANCE = 123,
    ValveController_GeneralMSG_CMD_FEEDFORWARD = 125,
    ValveController_GeneralMSG_CMD_BULKMODULUS = 126,
    ValveController_GeneralMSG_CMD_CHAMBERVOL = 127,
    ValveController_GeneralMSG_CMD_PISTONAREA = 128,
    ValveController_GeneralMSG_CMD_SUPnRETPRES = 129,
    ValveController_GeneralMSG_CMD_JOINTENCLIMIT = 130,
    ValveController_GeneralMSG_CMD_PISTONSTROKE = 131,
    ValveController_GeneralMSG_CMD_VALVEPOSLIMIT = 132,
    ValveController_GeneralMSG_CMD_ENCPPP = 133,
    ValveController_GeneralMSG_CMD_SENPPF = 134,
    ValveController_GeneralMSG_CMD_SENPPP = 135,
    ValveController_GeneralMSG_CMD_CONSTFRICTION = 136,
    ValveController_GeneralMSG_CMD_HOMEPOSOFFSET = 140,
    ValveController_GeneralMSG_CMD_HOMEPOSOPENING = 141,


    ValveController_GeneralMSG_CMD_ERRORCLEAR = 150,

};

void    RBCMD_ValveControllerRequestOnOff(void);
void    RBCMD_AskValveControllerParameters(int _BN);
void    RBCMD_SetValveControllerParameters(int _BN, HCB_INFO H);
void    RBCMD_ReadnSaveValveControllerParameters(void);
void    RBCMD_LoadnSetValveControllerParameters(void);

#endif // HydraulicActuatorController_H
