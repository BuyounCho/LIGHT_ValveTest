#include "RBIMUSensor.h"
#include <iostream>
using namespace std;

extern int     _NO_OF_COMM_CH;

RBIMUSensor::RBIMUSensor()
{
    ROLL_OFFSET = 0.0;
    PITCH_OFFSET = 0.0;
    YAW_OFFSET = 0.0;
}

void RBIMUSensor::RBBoard_GetDBData(DB_IMU db){
    BOARD_ID            = db.BOARD_ID;
    BOARD_NAME          = db.BOARD_NAME;
    CAN_CHANNEL         = db.CAN_CHANNEL;
    ID_SEND_DATA        = db.ID_SEND_DATA;
    ID_RCV_GENERAL      = db.ID_RCV_GENERAL;
    ID_RCV_DATA_QUAT    = db.ID_RCV_DATA_QUAT;
    ID_RCV_DATA_LOCAL_X = db.ID_RCV_DATA_LOCAL_X;
    ID_RCV_DATA_LOCAL_Y = db.ID_RCV_DATA_LOCAL_Y;
    ID_RCV_DATA_LOCAL_Z = db.ID_RCV_DATA_LOCAL_Z;
}

int RBIMUSensor::SendGeneralMSG(int _typeMSG, void *arg)
{
    ST_CAN mb;
    mb.id = ID_SEND_DATA;
    mb.data[0] = _typeMSG;

    switch(_typeMSG) {
    case IMU_GeneralMSG_CANCHECK:
        mb.dlc = 1;
        break;
    case IMU_GeneralMSG_REQUEST_ONOFF:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0]; // BNO
        break;
    }
    case IMU_GeneralMSG_RESET:
    {
        mb.dlc = 2;
        mb.data[1] = 0;
        break;
    }
    case IMU_GeneralMSG_NULLING:
        mb.dlc = 1;
        break;
    default:
        break;
    }
    rb_can::write_general_msg(mb, CAN_CHANNEL);
    usleep(1000);
}

int RBIMUSensor::CANCheck()
{
    ConnectionStatus = false; // For checking connection status, make it false once.
    SendGeneralMSG(IMU_GeneralMSG_CANCHECK);
    usleep(50*1000); // Waiting for Checking ConnectionStatus
    if(ConnectionStatus){
        std::cout << ">>> RBIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
        return true;
    } else {
        std::cout << ">>> RBIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
        return false;
    }
}

int  RBIMUSensor::CANChannel_Arrange(){
    std::cout << " This Function is not used. \n";

//    RBCAN_MB mb;
//    std::cout << " ======== Board(" << BOARD_ID << ": IMU) is checking channels... ========\n";

//    int NUM_CHECK_CAN_CH = min(MAX_HCB_CAN_CHANNEL,_NO_OF_COMM_CH);
//    for(int idx = 0; idx<NUM_CHECK_CAN_CH; idx++){
//        mb.data[0] = 0; //Ask Board status(information)
//        mb.dlc = 1;
//        mb.id = ID_SEND_DATA;
//        mb.channel = idx;
//        if(canHandler->RBCAN_WriteData(mb)){
//            usleep(100*1000);
//            mb.channel = idx;
//            mb.id = ID_RCV_GENERAL;
//            canHandler->RBCAN_ReadData(&mb);

//            if(mb.status != RBCAN_NODATA && mb.data[0]==0){
//                std::cout << ">>> Board(" << BOARD_ID << ": IMU) is \033[32mconnected \033[0mto channel [" << idx << "]\n";
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

//    std::cout << ">>> Board(" << BOARD_ID << ": IMU) is \033[31mnot connected.\033[0m \n";
//    std::cout << " ========================================================\n\n";
//    ConnectionStatus = false;
//    mb.status = RBCAN_NODATA;
//    return true;
}

int RBIMUSensor::RBIMU_RequestONOFF(int ONOFF) {
    int argInt[4] = {0,};
    argInt[0] = ONOFF;
    SendGeneralMSG(IMU_GeneralMSG_REQUEST_ONOFF, argInt);
}

int RBIMUSensor::RBIMU_Reset(void){
    SendGeneralMSG(IMU_GeneralMSG_RESET);
}

int RBIMUSensor::RBIMU_Nulling(void){
    SendGeneralMSG(IMU_GeneralMSG_NULLING);
}
