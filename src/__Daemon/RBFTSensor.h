#ifndef RBFTSENSOR_H
#define RBFTSENSOR_H

#include "RBDataType.h"
//#include "RBCAN.h"
#include "RBCAN_new.h"


class RBFTSensor
{
public:
    RBFTSensor();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     CAN_CHANNEL;
    int     ID_SEND_CMD;
    int     ID_RCV_GENERAL;
    int     ID_RCV_MXMYFZ;
    int     ID_RCV_FXFYMZ;


    // not from DB----
    int     ConnectionStatus;

    // sensor data----
    double   MX;
    double   MX_FILTERED;
    double   MY;
    double   MY_FILTERED;
    double   MZ;
    double   MZ_FILTERED;
    double   FX;
    double   FX_FILTERED;
    double   FY;
    double   FY_FILTERED;
    double   FZ;
    double   FZ_FILTERED;
    double   AccRoll;
    double   AccRollOld;
    double   dAccRoll;
    double   dAccRollOld;
    double   dAccRoll_Offset;
    double   AccPitch;
    double   AccPitchOld;
    double   dAccPitch;
    double   dAccPitchOld;
    double   dAccPitch_Offset;
    double   VelRoll;
    double   VelRollOld;
    double   VelPitch;
    double   VelPitchOld;
    double   ROLL;
    double   PITCH;

    // sensor setting----
    double   CutOffFeq;
    double   SFRoll;
    double   SFPitch;

    void    RBBoard_GetDBData(DB_FT db);

    int     SendGeneralMSG(int _typeMSG, void *arg = NULL);

    int     CANCheck();
    int     CANChannel_Arrange();
    int     RBFT_RequestONOFF(bool ONOFF);
    int     RBFT_Nulling(int _mode);

    int     RBFT_ReadData(void);
    int     RBBoard_RequestStatus(void);
    int     RBBoard_LoadDefaultValue(void);
    int     RBFT_SetCoefficient0(int _coeff1, int _coeff2, int _coeff3);
    int     RBFT_SetCoefficient1(int _coeff1, int _coeff2, int _coeff3);
    int     RBFT_SetCoefficient2(int _coeff1, int _coeff2, int _coeff3);
    int     RBFT_SetInclinometerSF(int _sf1, int _sf2, int _sf3);
    int     RBFT_SetBoardNumberAndFilterFrequency(int _newbno, int _freq);
    int     RBFT_Initialize(void);
    int     RBFT_RequestCoefficient(int _para);
    int     RBFT_RequestData(int _mode);

};

enum FT_GeneralMSGSET {
    FT_GeneralMSG_CANCHECK = 0,
    FT_GeneralMSG_REQUEST_ONOFF = 1,
    FT_GeneralMSG_NULLING = 2
};


#endif // RBFTSENSOR_H
