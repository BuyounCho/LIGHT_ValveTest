#ifndef RBIMUSENSOR_H
#define RBIMUSENSOR_H

#include "RBDataType.h"
//#include "RBCAN.h"
#include "RBCAN_new.h"

#include <cmath>

class RBIMUSensor
{
public:
    RBIMUSensor();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     CAN_CHANNEL;
    int     ID_SEND_DATA;
    int     ID_RCV_GENERAL;
    int     ID_RCV_DATA_QUAT;
    int     ID_RCV_DATA_LOCAL_X;
    int     ID_RCV_DATA_LOCAL_Y;
    int     ID_RCV_DATA_LOCAL_Z;

    // not from DB----
    int     ConnectionStatus;

    // sensor data----    
    // Current raw data   
    float     wx_local;
    float     wy_local;
    float     wz_local;
    float     ax_local;
    float     ay_local;
    float     az_local;

    // Offset value for angles
    double   ROLL_OFFSET;
    double   PITCH_OFFSET;
    double   YAW_OFFSET;

    double   FOG_ROLL_OFFSET;
    double   FOG_PITCH_OFFSET;
    double   FOG_YAW_OFFSET;

    double   FOG_ROLL_NULL;
    double   FOG_PITCH_NULL;
    double   FOG_YAW_NULL;

    void    RBBoard_GetDBData(DB_IMU db);

    int     SendGeneralMSG(int _typeMSG, void *arg = NULL);

    int     CANCheck();
    int     CANChannel_Arrange();
    int     RBIMU_RequestONOFF(int ONOFF);
    int     RBIMU_Reset(void);
    int     RBIMU_Nulling(void);

};

enum IMU_GeneralMSGSET {
    IMU_GeneralMSG_CANCHECK = 0,
    IMU_GeneralMSG_REQUEST_ONOFF = 1,
    IMU_GeneralMSG_RESET = 2,
    IMU_GeneralMSG_NULLING = 3
};



#endif // RBIMUSENSOR_H
