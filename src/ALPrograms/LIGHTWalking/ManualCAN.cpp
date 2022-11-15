#include "ManualCAN.h"
#include <iostream>

extern pRBCORE_SHM_COMMAND      sharedCMD;

#include <stdio.h>

int	PushCANMessage(MANUAL_CAN MCData);


int MCSendFindhome(unsigned int _canch, unsigned int _bno,  unsigned int _VCh);
int MCJointENCZero(unsigned int _canch, unsigned int _bno,  unsigned int _VCh);
//int MCSendPWMZero(unsigned int _canch, unsigned int _bno,  unsigned int _VCh);
int MCLoadCellsensorNull(unsigned int _canch, unsigned int _bno);

int MCJointSetControlMode(unsigned int _canch, unsigned int _bno, unsigned int _VCh, int _mode);
int MCSendPositionRef(unsigned int _canch, unsigned int _bno, short _pREF1, short _pREF2);
int MCSendVelocityRef(unsigned int _canch, unsigned int _bno, short _vREF1, short _vREF2);
int MCSendForceRef(unsigned int _canch, unsigned int _bno, short _fREF1, short _fREF2);
int MCSendSupplyPressure(unsigned int _canch, unsigned int _bno, short pres_s);
int MCSendOpenloopPWM(unsigned int _canch, unsigned int _bno, short PWM1, short PWM2);

int MC_IMU_RESET(void);
int MC_IMU_NULLING(void);
int MC_IMU_ENABLE(void);

// --------------------------------------------------------------------------------------------- //
int PushCANMessage(MANUAL_CAN MCData)
{
    for(int i=0; i<MAX_MANUAL_CAN; i++){
        if(sharedCMD->ManualCAN[i].status == MANUALCAN_EMPTY){
            sharedCMD->ManualCAN[i].status = MANUALCAN_WRITING;
            sharedCMD->ManualCAN[i].channel = MCData.channel;
            sharedCMD->ManualCAN[i].id = MCData.id;
            sharedCMD->ManualCAN[i].dlc = MCData.dlc;
            for(int j=0; j<MCData.dlc; j++){
                sharedCMD->ManualCAN[i].data[j] = MCData.data[j];
            }

            sharedCMD->ManualCAN[i].status = MANUALCAN_NEW;
            return 0;
        }
    }

    return 1;
}


///// --------------------------------------------------------------------------------------------- //
/////  Manual CAN Protocol for LIGHT (Buyoun & Sungwoo)
/////  2018/09/05
///// --------------------------------------------------------------------------------------------- //

////// Sensor Command //////

// Valve Opening Control
int MCSendFindhome(unsigned int _canch, unsigned int _bno,  unsigned int _VCh)
{
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 1;
    MCData.data[0] = 0x28;           // command

    return PushCANMessage(MCData);
}

int MCJointENCZero(unsigned int _canch, unsigned int _bno,  unsigned int _VCh)
{
    MANUAL_CAN MCData;
    MCData.channel = _canch;
    MCData.id =  0x210 + _bno;
    MCData.dlc = 2;
    MCData.data[0] = 0x06;
    MCData.data[1] = _VCh; //1,2,0

    return PushCANMessage(MCData);
}

//int MCSendPWMZero(unsigned int _canch, unsigned int _bno,  unsigned int _VCh)
//{
//    MANUAL_CAN MCData;
//    MCData.channel = _canch;
//    MCData.id =  0x210 + _bno;
//    MCData.dlc = 2;
//    MCData.data[0] = 0x00;
//    MCData.data[1] = _VCh; //1,2,0

//    return PushCANMessage(MCData);
//}


int MCLoadCellsensorNull(unsigned int _canch, unsigned int _bno)
{
    // hand -> 0= Right 1=left 2=both
    // MsgID                Byte0	Byte1   Byte2
    // CANID_TXDF   		BNO		0x81    _mode

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 3;

    MCData.data[0] = _bno;		// board no.
    MCData.data[1] = 0x06;		// command
    MCData.data[2] = 0x00;		// mode
    //mode = 0x00 : FT sensor
    //mode = 0x04 : Inclinometers in FT sensor

    return PushCANMessage(MCData);
}

////// Robot Control method and Sending Joint Reference  //////

int MCSendLIGHTControlMode(unsigned int _canch, unsigned int _bno, char mode)
{
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 2;
    MCData.data[0] = 0X10;           // command
    MCData.data[1] = (mode & 0x00FF);
    // mode 0 : hybrid control (position+force)
    // mode 1 : position control
    // mode 2 : force control

    return PushCANMessage(MCData);
}

int MCSendPositionRef(unsigned int _canch, unsigned int _bno, short _pREF1, short _pREF2)
{
    MANUAL_CAN MCData;
    MCData.id =  0x210 + _bno;
    MCData.channel = _canch;
    MCData.dlc = 5;
    MCData.data[0] = 0x11;
    MCData.data[1] = (_pREF1 & 0x00FF);
    MCData.data[2] = ((_pREF1>>8) & 0x00FF);
    MCData.data[3] = (_pREF2 & 0x00FF);
    MCData.data[4] = ((_pREF2>>8) & 0x00FF);

    return PushCANMessage(MCData);
}

int MCSendVelocityRef(unsigned int _canch, unsigned int _bno, short _vREF1, short _vREF2)
{
    MANUAL_CAN MCData;
    MCData.id =  0x210 + _bno;
    MCData.channel = _canch;
    MCData.dlc = 5;
    MCData.data[0] = 0x12;
    MCData.data[1] = (_vREF1 & 0x00FF);
    MCData.data[2] = ((_vREF1>>8) & 0x00FF);
    MCData.data[3] = (_vREF2 & 0x00FF);
    MCData.data[4] = ((_vREF2>>8) & 0x00FF);

    return PushCANMessage(MCData);
}

int MCSendForceRef(unsigned int _canch, unsigned int _bno, short _fREF1, short _fREF2)
{
    MANUAL_CAN MCData;
    MCData.id =  0x210 + _bno;
    MCData.channel = _canch;
    MCData.dlc = 5;
    MCData.data[0] = 0x13;
    MCData.data[1] = (_fREF1 & 0x00FF);
    MCData.data[2] = ((_fREF1>>8) & 0x00FF);
    MCData.data[3] = (_fREF2 & 0x00FF);
    MCData.data[4] = ((_fREF2>>8) & 0x00FF);

    return PushCANMessage(MCData);
}

int MCSendSupplyPressure(unsigned int _canch, unsigned int _bno, short pres_s)
{
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 3;
    MCData.data[0] = 0x14;           // command
    MCData.data[1] = (pres_s & 0x00FF);
    MCData.data[2] = ((pres_s>>8) & 0x00FF);

    return PushCANMessage(MCData);
}

int MCSendOpenloopPWM(unsigned int _canch, unsigned int _bno, short PWM1, short PWM2)
{
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 5;
    MCData.data[0] = 0x15;           // command
    MCData.data[1] = (PWM1 & 0x00FF);
    MCData.data[2] = ((PWM1>>8) & 0x00FF);
    MCData.data[3] = (PWM2 & 0x00FF);
    MCData.data[4] = ((PWM2>>8) & 0x00FF);

    return PushCANMessage(MCData);
}


////// Operational Command  //////


////// IMU Command  //////

int MC_IMU_RESET(void)
{
    MANUAL_CAN MCData;
    MCData.id =  150;
    MCData.channel = 1;
    MCData.data[0] = 2;
    MCData.data[1] = 0;
    MCData.dlc = 2;

    return PushCANMessage(MCData);
}

int MC_IMU_NULLING(void)
{
    MANUAL_CAN MCData;
    MCData.id =  150;
    MCData.channel = 1;
    MCData.data[0] = 3;
    MCData.dlc = 1;

    return PushCANMessage(MCData);
}

int MC_IMU_ENABLE(void)
{
    MANUAL_CAN MCData;
    MCData.id =  150;
    MCData.channel = 1;
    MCData.data[0] = 1;
    MCData.data[1] = 1;
    MCData.dlc = 2;

    return PushCANMessage(MCData);
}






