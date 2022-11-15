#include "RBFTSensor.h"
#include <iostream>
using namespace std;

extern int     _NO_OF_COMM_CH;

RBFTSensor::RBFTSensor()
{
    CutOffFeq = 3.0f;
    SFRoll = 1.0f/0.0255f;
    SFPitch = 1.0f/0.0255f;
}

void RBFTSensor::RBBoard_GetDBData(DB_FT db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_NAME      = db.BOARD_NAME;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    ID_SEND_CMD     = db.ID_SEND_CMD;
    ID_RCV_GENERAL  = db.ID_RCV_GENERAL;
    ID_RCV_MXMYFZ   = db.ID_RCV_MXMYFZ;
    ID_RCV_FXFYMZ   = db.ID_RCV_FXFYMZ;
}

int RBFTSensor::SendGeneralMSG(int _typeMSG, void *arg)
{
    ST_CAN mb;

    switch(_typeMSG) {
    case FT_GeneralMSG_CANCHECK:
    {
        mb.id = COMMAND_CANID;
        mb.data[0] = BOARD_ID;      // board no.
        mb.data[1] = 0x01;          // CAN Check CMD
        mb.dlc = 2;
        break;
    }
    case FT_GeneralMSG_REQUEST_ONOFF:
    {
        mb.id = ID_SEND_CMD;
        mb.data[0] = BOARD_ID;      // board no.
        mb.data[1] = 0x02;			// Sensor Enable/Disable

        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[2] = temp[0]; // 1 : Enable, 0 : Disable
        mb.dlc = 3;
        break;
    }
    case FT_GeneralMSG_NULLING:
    {
        mb.id = ID_SEND_CMD;
        mb.data[0] = BOARD_ID;      // board no.
        mb.data[1] = 0x81;			// Null Command
        mb.data[2] = 0;             // Nulling Mode
        // _mode = 0x00 : FT sensor
        // _mode = 0x04 : Inclinometers in FT sensor
        mb.dlc = 3;
        break;
    }
    default:
        break;
    }
    rb_can::write_general_msg(mb, CAN_CHANNEL);
    usleep(1000);
}


int RBFTSensor::CANCheck()
{
    ConnectionStatus = false; // For checking connection status, make it false once.
    SendGeneralMSG(FT_GeneralMSG_CANCHECK);
    usleep(50*1000); // Waiting for Checking ConnectionStatus
    if(ConnectionStatus){
        std::cout << ">>> RBFT: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
        return true;
    } else {
        std::cout << ">>> RBFT: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
        return false;
    }
}

int  RBFTSensor::CANChannel_Arrange(){
    std::cout << " This Function is not used. \n";

//    RBCAN_MB mb;
//    std::cout << " ======== Board(" << BOARD_ID << ": FT) is checking channels... ========\n";

//    int NUM_CHECK_CAN_CH = min(MAX_HCB_CAN_CHANNEL,_NO_OF_COMM_CH);
//    for(int idx = 0; idx<NUM_CHECK_CAN_CH; idx++){
//        mb.id = COMMAND_CANID;
//        mb.channel = idx;
//        mb.data[0] = BOARD_ID; //Ask Board status(information)
//        mb.data[1] = 0x01;		// and
//        mb.dlc = 2;
//        if(canHandler->RBCAN_WriteData(mb)){
//            usleep(100*1000);
//            mb.channel = idx;
//            mb.id = ID_RCV_GENERAL;
//            canHandler->RBCAN_ReadData(&mb);

//            if(mb.status != RBCAN_NODATA && mb.data[0]==0){
//                std::cout << ">>> Board(" << BOARD_ID << ": FT) is \033[32mconnected \033[0mto channel [" << idx << "]\n";
//                std::cout << " ========================================================\n\n";

//                CAN_CHANNEL = idx;
//                ConnectionStatus = true;
//                mb.status = RBCAN_NODATA;
//                return true;
//            }else{
//                std::cout << "\033[31m >>> [ Channel " << idx << " ] is not connected... \033[0m " << std::endl;
//            }
//        } else {
//            std::cout << "\033[31m >>> [ Channel " << idx << " ] is not connected... \033[0m " << std::endl;
//        }
//    }

//    std::cout << ">>> Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mnot connected.\033[0m \n";
//    std::cout << " ========================================================\n\n";
//    ConnectionStatus = false;
//    mb.status = RBCAN_NODATA;
//    return true;
}

int RBFTSensor::RBFT_RequestONOFF(bool ONOFF){
    int argInt[4] = {0,};
    argInt[0] = ONOFF;
    SendGeneralMSG(FT_GeneralMSG_REQUEST_ONOFF, argInt);
}

int RBFTSensor::RBFT_Nulling(int _mode){
    SendGeneralMSG(FT_GeneralMSG_NULLING);
}


////////////////////////////////////////////////////////////////////////////////////////////////
//// NOT USED IN LIGHT
//////////////////////////////////////////////////////////////////////////////////////////////

/*

int RBFTSensor::RBFT_ReadData(void){
    int ret = true;
    RBCAN_MB mb;

    // Read MX MY FZ
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_MXMYFZ;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){

//        MX = -(double)((short)((mb.data[1]<<8)|mb.data[0]));
//        MY = -(double)((short)((mb.data[3]<<8)|mb.data[2]));
//        FZ = -(double)((short)((mb.data[5]<<8)|mb.data[4]))/10.0f;
        MX = -(double)((short)((mb.data[2]<<8)|mb.data[1]))/100.0f;
        MY = -(double)((short)((mb.data[4]<<8)|mb.data[3]))/100.0f;
        FZ = -(double)((short)((mb.data[6]<<8)|mb.data[5]))/10.0f;

        //printf("FT %4d %4d %4d %4d %4d %4d \n",mb.data[1],mb.data[2],mb.data[3],mb.data[4],mb.data[5],mb.data[6]);

        MX_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*MX_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*MX;
        MY_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*MY_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*MY;
        FZ_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*FZ_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*FZ;
        mb.status = RBCAN_NODATA;
    } else {
        ret = false;
    }

    // Read FX FY MZ
    mb.id = ID_RCV_FXFYMZ;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        FX = -(double)((short)((mb.data[2]<<8)|mb.data[1]))/10.0f;
        FY = -(double)((short)((mb.data[4]<<8)|mb.data[3]))/10.0f;
        MZ = -(double)((short)((mb.data[6]<<8)|mb.data[5]))/100.0f;

        FX_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*FX_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*FX;
        FY_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*FY_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*FY;
        MZ_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*MZ_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*MZ;
        mb.status = RBCAN_NODATA;
    } else {
        ret = false;
    }

    return ret;
}

int RBFTSensor::RBBoard_RequestStatus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x02;		// and
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBBoard_LoadDefaultValue(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xFA;			// and
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetCoefficient0(int _coeff1, int _coeff2, int _coeff3){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA0;						// and
    mb.data[2] = (_coeff1 & 0xFF);			// scale factor 00
    mb.data[3] = ((_coeff1>>8) & (0xFF));	// scale factor 00
    mb.data[4] = (_coeff2 & 0xFF);			// scale factor 01
    mb.data[5] = ((_coeff2>>8) & (0xFF));	// scale factor 01
    mb.data[6] = (_coeff3 & 0xFF);			// scale factor 02
    mb.data[7] = ((_coeff3>>8) & (0xFF));	// scale factor 02
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetCoefficient1(int _coeff1, int _coeff2, int _coeff3){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA1;						// and
    mb.data[2] = (_coeff1 & 0xFF);			// scale factor 10
    mb.data[3] = ((_coeff1>>8) & (0xFF));	// scale factor 10
    mb.data[4] = (_coeff2 & 0xFF);			// scale factor 11
    mb.data[5] = ((_coeff2>>8) & (0xFF));	// scale factor 11
    mb.data[6] = (_coeff3 & 0xFF);			// scale factor 12
    mb.data[7] = ((_coeff3>>8) & (0xFF));	// scale factor 12
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetCoefficient2(int _coeff1, int _coeff2, int _coeff3){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA2;						// and
    mb.data[2] = (_coeff1 & 0xFF);			// scale factor 20
    mb.data[3] = ((_coeff1>>8) & (0xFF));	// scale factor 20
    mb.data[4] = (_coeff2 & 0xFF);			// scale factor 21
    mb.data[5] = ((_coeff2>>8) & (0xFF));	// scale factor 21
    mb.data[6] = (_coeff3 & 0xFF);			// scale factor 22
    mb.data[7] = ((_coeff3>>8) & (0xFF));	// scale factor 22
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetInclinometerSF(int _sf1, int _sf2, int _sf3){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA5;						// and
    mb.data[2] = (_sf1 & 0xFF);			// scale factor 20
    mb.data[3] = ((_sf1>>8) & (0xFF));		// scale factor 20
    mb.data[4] = (_sf2 & 0xFF);			// scale factor 21
    mb.data[5] = ((_sf2>>8) & (0xFF));		// scale factor 21
    mb.data[6] = (_sf3 & 0xFF);			// scale factor 22
    mb.data[7] = ((_sf3>>8) & (0xFF));		// scale factor 22
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetBoardNumberAndFilterFrequency(int _newbno, int _freq){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA8;						// and
    mb.data[2] = _newbno;					// new board number
    mb.data[4] = (_freq & 0xFF);	    	// low-pass filter(1st order) cut-off frequency
    mb.data[5] = ((_freq>>8) & (0xFF));	// low-pass filter(1st order) cut-off frequency
    // cut-off frequency = _freq/10
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_Initialize(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xFA;						// and
    mb.data[2] = 0xAA;						// and
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_RequestCoefficient(int _para){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0x24;						// and
    mb.data[2] = _para;					// parameter request
    // _para = 0x01 : SF00 SF01 SF02 FREQ
    // _para = 0x02 : SF10 SF11 SF12
    // _para = 0x03 : SF20 SF21 SF22
    // _para = 0x04 : SFI0 SFI1 SFI2
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_RequestData(int _mode){
//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;
//    mb.data[0] = SENSOR_ID;					// sensor board no. if _sbno = 0xFF : all sensor boards
//    mb.data[1] = _mode;					// requested data mode
//    // _mode = 0x00 : request FT and tilt in digit
//    // _mode = 0x02 : request FT and tilt with scale
//    // _mode = 0x03 : request FT with scale and tilt in digit
//    // _mode = 0x04 : request FT in digit and tilt with scale
//    // _mode = 0x11 : request FT in digit
//    // _mode = 0x12 : request FT with scale
//    // _mode = 0x21 : request tilt in digit
//    // _mode = 0x22 : request tilt with scale
//    // _mode = 0x13 : request gyro and temperature
//    mb.dlc = 2;
//    mb.id = SENSOR_REQUEST_CANID;
//    if(SENSOR_TYPE==101)//oldsensor
//    {
//        mb.id = SENSOR_REQUEST_CANID-1;//0x02
//    }

//    return canHandler->RBCAN_WriteDataDirectly(mb);
}

*/
