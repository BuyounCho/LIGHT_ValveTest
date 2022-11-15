#ifndef RBDATATYPE_H
#define RBDATATYPE_H

#include <QString>
#include <unistd.h>
//#include <math.h>
#include <cmath>
#include "../../share/Headers/RBSharedMemory.h"


typedef	unsigned int	uint;
typedef unsigned char	uchar;
typedef	unsigned long	ulong;

//typedef class RBCAN     *pRBCAN;
typedef class RBRawLAN  *pRBLAN;
extern pRBCORE_SHM_COMMAND      sharedCMD;
extern pRBCORE_SHM_REFERENCE    sharedREF;
extern pRBCORE_SHM_SENSOR       sharedSEN;
//extern pRBCAN                   canHandler;


//======================================================
// Database data type for General
typedef struct _DB_GENERAL_{
    int     VERSION;
    int     NO_OF_AL;
    int     NO_OF_COMM_CH;      // communication channel
    int     NO_OF_VC;           // motor controller
    int     NO_OF_PC;           // pump controller
    int     NO_OF_FT;           // ft sensor
    int     NO_OF_IMU;          // imu sensor
    int     NO_OF_SP;           // smart power controller
    int     NO_OF_OF;           // optic flow sensor
}DB_GENERAL;

// Database data type for Motor Controller
typedef struct _DB_VC_{
    QString BOARD_NAME;
    int     BOARD_ID;
    int     CAN_CHANNEL;
    QString ACTUATOR_TYPE;
    QString VALVE_TYPE;

    double  PULSE_PER_POSITION;
    double  PULSE_PER_FORCETORQUE;
    double  PULSE_PER_PRESSURE;

    int     ID_SEND_GENERAL;
    int     ID_SEND_POSVEL;
    int     ID_SEND_VALVEPOSnPWM;

    int     ID_RCV_GENERAL;
    int     ID_RCV_POSVEL;
    int     ID_RCV_VALVEPOSnPWM;
    int     ID_RCV_PRESSURE;
    int     ID_RCV_OTHERINFO;
    int     ID_RCV_ALART;

}DB_VC;

// Database data type for FT Sensor
typedef struct _DB_FT_{
    int     BOARD_ID;
    QString BOARD_NAME;
    int     CAN_CHANNEL;
    int     ID_SEND_CMD;
    int     ID_RCV_GENERAL;
    int     ID_RCV_MXMYFZ;
    int     ID_RCV_FXFYMZ;
}DB_FT;

// Database data type for IMU Sensor
typedef struct _DB_IMU_{
    QString BOARD_NAME;
    int     BOARD_ID;
    int     CAN_CHANNEL;
    int     ID_SEND_DATA;
    int     ID_RCV_GENERAL;
    int     ID_RCV_DATA_QUAT;
    int     ID_RCV_DATA_LOCAL_X;
    int     ID_RCV_DATA_LOCAL_Y;
    int     ID_RCV_DATA_LOCAL_Z;
}DB_IMU;

// Database data type for Smart Power Controller
typedef struct _DB_SP_{
    QString BOARD_NAME;
    int     BOARD_ID;
    int     CAN_CHANNEL;
    int     ID_RCV_DATA;
    int     ID_RCV_INFO;
    int     ID_SEND_GENERAL;
}DB_SP;

// Database data type for Optic Flow Sensor
typedef struct _DB_OF_{
    QString BOARD_NAME;
    int     BOARD_ID;
    int     SENSOR_ID;
    int     CAN_CHANNEL;
    int     ID_RCV_DATA;
    int     ID_RCV_INFO;
}DB_OF;

// Database data type for PODO AL
typedef struct _DB_AL_{
    QString ALName;
    QString FileName;
    QString PathName;
}DB_AL;

typedef struct _DB_PC_{
    int     BOARD_ID;
    int     CAN_CHANNEL;

    int     ID_SEND_GENERAL;
    int     ID_SEND_VELOCITY;
    int     ID_SEND_PRESSURE;
    int     ID_RCV_GENERAL;
    int     ID_RCV_VELOCITY;
    int     ID_RCV_PRESSURE;
}DB_PC;
//======================================================


//======================================================
typedef struct _RBCORE_JOINT_
{
    //Hydraulic Control Board Data
    HCB_REF         HCB_Ref;
    HCB_DATA        HCB_Data;
    HCB_INFO        HCB_Info;
} RB_JOINT, *pRB_JOINT;
//======================================================
#endif // RBDATATYPE_H
