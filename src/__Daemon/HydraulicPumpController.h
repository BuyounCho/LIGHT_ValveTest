#ifndef HYDRAULICPUMPCONTROLLER_H
#define HYDRAULICPUMPCONTROLLER_H

#include "RBDataType.h"
//#include "RBCAN.h"
#include "RBCAN_new.h"
//#include "RBSPI2CAN.h"
#include <cmath>

class PumpController
{
public:
    PumpController();

    // from DB <-
    int     BOARD_ID;
    int     CAN_CHANNEL;
    bool    REQUEST_ONOFF_DATA;

    int     ID_SEND_GENERAL;
    int     ID_SEND_VELOCITY;

    int     ID_RCV_GENERAL;
    int     ID_RCV_VELOCITY;
    int     ID_RCV_PRESSURE;

    // not from DB----
    int     ConnectionStatus;
    double   CurrentVelocity;     // Pump Rotation Speed
    double   CurrentPressure;     // Supply Pressure
    double   CurrentTemperature;  // Oil(or Wire) Temperature

    double  ReferencePumpVelocity;
    double  ReferencePumpVelocity_last;

    void    GetDBData(DB_PC db);

    int SendGeneralMSG(int _typeMSG, void *arg = NULL);

    // Board Operation Setting ///////////////////////////////////////////////
    int CANCheck(void);
    int CANChannel_Arrange(void);

};


#endif // HYDRAULICPUMPCONTROLLER_H
