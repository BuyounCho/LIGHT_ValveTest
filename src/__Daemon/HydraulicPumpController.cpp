#include "HydraulicPumpController.h"
#include <iostream>
using namespace std;

#include <chrono>
using namespace std::chrono;

extern int     _NO_OF_COMM_CH;

PumpController::PumpController() {
    REQUEST_ONOFF_DATA = false;
    ReferencePumpVelocity = 0.0;
}

void PumpController::GetDBData(DB_PC db) {
    BOARD_ID = db.BOARD_ID;
    CAN_CHANNEL = db.CAN_CHANNEL;

    ID_SEND_GENERAL = db.ID_SEND_GENERAL;
    ID_SEND_VELOCITY = db.ID_SEND_VELOCITY;

    ID_RCV_GENERAL = db.ID_RCV_GENERAL;
    ID_RCV_VELOCITY = db.ID_RCV_VELOCITY;
    ID_RCV_PRESSURE = db.ID_RCV_PRESSURE;
}

int PumpController::SendGeneralMSG(int _typeMSG, void *arg)
{
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = _typeMSG;

    switch(_typeMSG) {
    case PumpController_GeneralMSG_CANCHECK:
        mb.dlc = 1;
        break;
    case PumpController_GeneralMSG_ASK_SPEEDREF:
        mb.dlc = 1;
        break;
    case PumpController_GeneralMSG_ASK_CTRL_ONOFF:
        mb.dlc = 1;
        break;
    case PumpController_GeneralMSG_ASK_DATAREQUESTFLAG:
        mb.dlc = 1;
        break;

    // ===================================================================
    case PumpController_GeneralMSG_CMD_SPEEDREF:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0x000000FF);
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0x000000FF);
        break;
    }
    case PumpController_GeneralMSG_CMD_CTRL_ON:
    {
        mb.dlc = 1;
        break;
    }
    case PumpController_GeneralMSG_CMD_CTRL_OFF:
    {
        mb.dlc = 1;
        break;
    }
    case PumpController_GeneralMSG_CMD_PRESSURENULL:
    {
        mb.dlc = 1;
        break;
    }
    case PumpController_GeneralMSG_CMD_DATAREQUESTFLAG:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0]; // ONOFF
        break;
    }
    default:
        break;
    }
    rb_can::write_general_msg(mb, CAN_CHANNEL);
//    usleep(1000);
}



/****************************************************************
 *  PumpController - CAN DATA TX
 ****************************************************************/

int PumpController::CANCheck(void)
{
    ConnectionStatus = false; // For checking connection status, make it false once.
    SendGeneralMSG(PumpController_GeneralMSG_CANCHECK);
    usleep(50*1000); // Waiting for Checking ConnectionStatus
    if(ConnectionStatus){
        std::cout << ">>> PC: Board(" << BOARD_ID << ") is \033[32mconnected.\033[0m[ch " << CAN_CHANNEL << "]\n";
        return true;
    } else {
        std::cout << ">>> PC: Board(" << BOARD_ID << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        return false;
    }
}

int  PumpController::CANChannel_Arrange(void){
//    RBCAN_MB mb;
//    std::cout << " ======== Board(" << BOARD_ID << ": PUMP) is checking channels... ========\n";

//    int NUM_CHECK_CAN_CH = min(MAX_HCB_CAN_CHANNEL,_NO_OF_COMM_CH);
//    for(int idx = 0; idx<NUM_CHECK_CAN_CH; idx++){
//        mb.data[0] = 0; //Ask Board status(information)
//        mb.dlc = 1;
//        mb.id = ID_SEND_GENERAL;
//        mb.channel = idx;
//        if(canHandler->RBCAN_WriteData(mb)){
//            usleep(100*1000);
//            mb.channel = idx;
//            mb.id = ID_RCV_GENERAL;
//            canHandler->RBCAN_ReadData(&mb);

//            if(mb.status != RBCAN_NODATA && mb.data[0]==0){
//                std::cout << ">>> Board(" << BOARD_ID << ": PUMP) is \033[32mconnected \033[0mto channel [" << idx << "]\n";
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

//    std::cout << ">>> Board(" << BOARD_ID << ": PUMP) is \033[31mnot connected.\033[0m \n";
//    std::cout << " ========================================================\n\n";
//    ConnectionStatus = false;
//    mb.status = RBCAN_NODATA;
//    return true;
}

