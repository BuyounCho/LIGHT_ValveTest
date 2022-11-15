#include "HydraulicActuatorController.h"

using namespace std;
using namespace std::chrono;

extern int     _NO_OF_COMM_CH;
extern int     _NO_OF_VC;

extern ValveController             _DEV_VC[MAX_VC];

extern int argInt[4]; // Argument for SendGeneralMSG Function
extern double argDouble[4]; // Argument for SendGeneralMSG Function


ValveController::ValveController()
{
    ConnectionStatus = false;
    ErrorFlag = false;
}

void ValveController::GetDBData(DB_VC db){
    BOARD_NAME  = db.BOARD_NAME;
    BOARD_ID    = db.BOARD_ID;
    CAN_CHANNEL = db.CAN_CHANNEL;
    ACTUATOR_TYPE = db.ACTUATOR_TYPE;
    VALVE_TYPE  = db.VALVE_TYPE;

    PULSE_PER_POSITION = db.PULSE_PER_POSITION;
    PULSE_PER_FORCETORQUE = db.PULSE_PER_FORCETORQUE;
    PULSE_PER_PRESSURE = db.PULSE_PER_PRESSURE;

    ID_SEND_GENERAL     = db.ID_SEND_GENERAL;
    ID_SEND_POSVEL      = db.ID_SEND_POSVEL;
    ID_SEND_VALVEPOSnPWM = db.ID_SEND_VALVEPOSnPWM;

    ID_RCV_GENERAL      = db.ID_RCV_GENERAL;
    ID_RCV_POSVEL       = db.ID_RCV_POSVEL;
    ID_RCV_VALVEPOSnPWM = db.ID_RCV_VALVEPOSnPWM;
    ID_RCV_PRESSURE     = db.ID_RCV_PRESSURE;
    ID_RCV_OTHERINFO    = db.ID_RCV_OTHERINFO;
    ID_RCV_ALART        = db.ID_RCV_ALART;
}

int ValveController::SendGeneralMSG(int _typeMSG, void *arg)
{
    ST_CAN mb;
    mb.id = ID_SEND_GENERAL;
    mb.data[0] = _typeMSG;

    switch(_typeMSG) {
    case ValveController_GeneralMSG_CANCHECK:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_BOARDNUMBER:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_BOARDOPERATIONMODE:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_CANFREQ:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_CTRLMODE:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_JOINTENCDIR:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VALVEINPUTDIR:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VALVEENCDIR:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_BOARDINPUTVOLTAGE:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VALVEOPERVOLTAGE:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VARIABLESUPPLY_ONOFF:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_PIDGAIN:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0];
        break;
    }
    case ValveController_GeneralMSG_ASK_VALVEDZ:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VELCOMP:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_COMPLIANCE:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_FEEDFORWARD:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_BULKMODULUS:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_CHAMBERVOL:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_PISTONAREA:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_SUPnRETPRES:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_JOINTENCLIMIT:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_PISTONSTROKE:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VALVEPOSLIMIT:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_ENCPPP:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_SENPPF:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_SENPPP:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_CONSTFRICTION:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VALVEGAINPLUS:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VALVEGAINMINUS:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_HOMEPOSOFFSET:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_HOMEPOSOPENING:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_ASK_VOLTAGE2VALVEPOS_RESULT:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0]; // index
        break;
    }
    case ValveController_GeneralMSG_ASK_VALVEPOS2FLOWRATE_RESULT:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0]; // index
        break;
    }
    // ===================================================================
    case ValveController_GeneralMSG_CMD_BOARDNUMBER:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0]; // BNO
        break;
    }
    case ValveController_GeneralMSG_CMD_BOARDOPERATIONMODE:
    {
        int *temp = (int*)arg;
        mb.dlc = 5;
        mb.data[1] = temp[0]; // (0 : Moog & Rot, 1 : Moog & Lin, 2 : KNR & Rot, 3 : KNR & Lin)
        mb.data[2] = temp[1]; // (0 : torque, 1: pressure)
        mb.data[3] = temp[2]; // (0 : pwm, 1 : current control)
        mb.data[4] = temp[3]; // (0 : not use mechanical deadzone, 1 : use)
        break;
    }
    case ValveController_GeneralMSG_CMD_ENCZERO:
        mb.dlc = 1;
        break;
    case ValveController_GeneralMSG_CMD_FETONOFF:
    {
        int *temp = (int*)arg;
        for(int j=0; j<MAX_JOINT; j++) Joints[j].HCB_Info.FET_ONOFF = (bool)temp[0];
        mb.dlc = 2;
        mb.data[1] = temp[0]; // (0 : OFF, 1: ON)
        break;
    }
    case ValveController_GeneralMSG_CMD_MODETRANSITION:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0]; // (ForceControl / PositionControl)
        break;
    }
    case ValveController_GeneralMSG_CMD_CANFREQ:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // [Hz]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_CTRLMODE:
    {
        int *temp = (int*)arg;
        mb.dlc = 2;
        mb.data[1] = temp[0];
        // data[1] = 0   : NO control mode
        // data[1] = 1   : valve open loop control mode
        // data[1] = 2   : valve position control mode
        // data[1] = 3   : joint position and force control based on PWM
        // data[1] = 20  : (force sensor or pressure sensor) nulling
        // data[1] = 22  : find home
        // data[1] = 30  : Voltage Input vs Valve Pos Identification (only DDV)
        // data[1] = 31  : Dead-Zone Identification
        // data[1] = 32  : Valve Pos vs Flowrate Identification
        break;
    }
    case ValveController_GeneralMSG_CMD_DATAREQUEST_ONOFF:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = temp[0]; // (0 : OFF, 1: ON)
        mb.data[2] = temp[1]; // 0 : PosVelTor, 1 : ValvePos, 2 : OtherInfo
        break;
    }
    case ValveController_GeneralMSG_CMD_JOINTENCDIR:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // +1 or -1
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_VALVEINPUTDIR:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // +1 or -1
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_VALVEENCDIR:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // +1 or -1
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_BOARDINPUTVOLTAGE:
    {
        double *temp = (double*)arg;
        int tempV = (int)(10.0*temp[0]);
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(tempV & 0xFF);
        mb.data[2] = (uint8_t)((tempV>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_VALVEOPERVOLTAGE:
    {
        double *temp = (double*)arg;
        int tempV = (int)(10.0*temp[0]);
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(tempV & 0xFF);
        mb.data[2] = (uint8_t)((tempV>>8) & 0xFF);
        break;
    }

    case ValveController_GeneralMSG_CMD_VARIABLESUPPLY_ONOFF:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = temp[0]; // (0 : OFF, 1: ON)
        mb.data[2] = temp[1]; // 0 : PosVelTor, 1 : ValvePos, 2 : OtherInfo
        break;
    }
    case ValveController_GeneralMSG_CMD_PIDGAIN:
    {
        double *temp = (double*)arg;
        mb.data[1] = (unsigned char)temp[0];
        // data[1] = 0 : Valve Position Gain
        // data[1] = 1 : Joint Position Gain
        // data[1] = 2 : Joint Force/torque Gain
        // data[1] = 3 : Joint Spring&Damper

        if(mb.data[1]==0||mb.data[1]==1||mb.data[1]==2) {
            mb.dlc = 8;
            int16_t tempP = (int16_t)(temp[1]);
            int16_t tempI = (int16_t)(temp[2]);
            int16_t tempD = (int16_t)(temp[3]);
            mb.data[2] = (unsigned char)(tempP & 0x000000FF);
            mb.data[3] = (unsigned char)((tempP>>8) & 0x000000FF);
            mb.data[4] = (unsigned char)(tempI & 0x000000FF);
            mb.data[5] = (unsigned char)((tempI>>8) & 0x000000FF);
            mb.data[6] = (unsigned char)(tempD & 0x000000FF);
            mb.data[7] = (unsigned char)((tempD>>8) & 0x000000FF);
        } else if(mb.data[1]==3) {
            mb.dlc = 6;
            int16_t tempK = (int16_t)(temp[1]*10.0);
            int16_t tempD = (int16_t)(temp[2]*100.0);
            mb.data[2] = (unsigned char)(tempK & 0x000000FF);
            mb.data[3] = (unsigned char)((tempK>>8) & 0x000000FF);
            mb.data[4] = (unsigned char)(tempD & 0x000000FF);
            mb.data[5] = (unsigned char)((tempD>>8) & 0x000000FF);
        }
        break;
    }
    case ValveController_GeneralMSG_CMD_VALVEDZ:
    {
        int *temp = (int*)arg;
        mb.dlc = 7;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // VALVE_CENTER_POS
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        mb.data[3] = (uint8_t)(temp[1] & 0xFF); // VALVE_DZ_PLUS
        mb.data[4] = (uint8_t)((temp[1]>>8) & 0xFF);
        mb.data[5] = (uint8_t)(temp[2] & 0xFF); // VALVE_DZ_MINUS
        mb.data[6] = (uint8_t)((temp[2]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_VELCOMP:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Compensation [%]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_COMPLIANCE:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Compliance Gain [Unit : ?]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_FEEDFORWARD:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Unit : PWM(two-stage valve) / Pulse(DDV)
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_BULKMODULUS:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Bulk Modulus [bar]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_CHAMBERVOL:
    {
        int *temp = (int*)arg;
        mb.dlc = 5;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Volume A [mm^3]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        mb.data[3] = (uint8_t)(temp[1] & 0xFF); // Volume B [mm^3]
        mb.data[4] = (uint8_t)((temp[1]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_PISTONAREA:
    {
        int *temp = (int*)arg;
        mb.dlc = 5;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Area A [mm^2]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        mb.data[3] = (uint8_t)(temp[1] & 0xFF); // Area B [mm^2]
        mb.data[4] = (uint8_t)((temp[1]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_SUPnRETPRES:
    {
        int *temp = (int*)arg;
        mb.dlc = 5;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Supply [bar]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        mb.data[3] = (uint8_t)(temp[1] & 0xFF); // Return [bar]
        mb.data[4] = (uint8_t)((temp[1]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_JOINTENCLIMIT:
    {
        int *temp = (int*)arg;
        mb.dlc = 5;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Minus Limit [pulse]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        mb.data[3] = (uint8_t)(temp[1] & 0xFF); // Plus Limit [pulse]
        mb.data[4] = (uint8_t)((temp[1]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_PISTONSTROKE:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Stroke [mm(linear) / deg(rotary)]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_VALVEPOSLIMIT:
    {
        int *temp = (int*)arg;
        mb.dlc = 5;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Plus Limit [pulse]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        mb.data[3] = (uint8_t)(temp[1] & 0xFF); // Minus Limit [pulse]
        mb.data[4] = (uint8_t)((temp[1]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_ENCPPP:
    {
        double *temp = (double*)arg;
        int tempG = (int)(1.0*temp[0]);
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(tempG & 0xFF); // Encoder Pulse per Position [Pulse/mm(Linear) / Pulse/deg(Rotary)]
        mb.data[2] = (uint8_t)((tempG>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_SENPPF:
    {
        double *temp = (double*)arg;
        int tempG = (int)(1000.0*temp[0]);
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(tempG & 0xFF); // Force Sensor Pulse per Force [Pulse/N(Linear) / Pulse/Nm(Rotary)]
        mb.data[2] = (uint8_t)((tempG>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_SENPPP:
    {
        double *temp = (double*)arg;
        int tempA = (int)(100.0*temp[0]);
        int tempB = (int)(100.0*temp[1]);
        mb.dlc = 5;
        mb.data[1] = (uint8_t)(tempA & 0xFF); // Pressure Sensor A Pulse per Force [Pulse/N(Linear) / Pulse/Nm(Rotary)]
        mb.data[2] = (uint8_t)((tempA>>8) & 0xFF);
        mb.data[3] = (uint8_t)(tempB & 0xFF); // Pressure Sensor B Pulse per Force [Pulse/N(Linear) / Pulse/Nm(Rotary)]
        mb.data[4] = (uint8_t)((tempB>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_CONSTFRICTION:
    {
        double *temp = (double*)arg;
        int tempF = (int)(10.0*temp[0]);
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(tempF & 0xFF); // Friction [N(Linear) / Nm(Rotary)]
        mb.data[2] = (uint8_t)((tempF>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_HOMEPOSOFFSET:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Home Pose Encoder Offset [pulse]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_HOMEPOSOPENING:
    {
        int *temp = (int*)arg;
        mb.dlc = 3;
        mb.data[1] = (uint8_t)(temp[0] & 0xFF); // Home Pose Valve Opening [pulse]
        mb.data[2] = (uint8_t)((temp[0]>>8) & 0xFF);
        break;
    }
    case ValveController_GeneralMSG_CMD_ERRORCLEAR:
        mb.dlc = 1;
        break;
    default:
        FILE_LOG(logERROR) << "Valve Controller (ID : "<< BOARD_ID << ") : Wrong General MSG ID.";
        return false;
    }
    rb_can::write_general_msg(mb, CAN_CHANNEL);
//    usleep(1000);
}



void RBCMD_ValveControllerRequestOnOff(void)
{
    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    bool enable = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];

    int type = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[3];

    if(id==-1)
    {
        if(enable) cout << "Request to Get Data from All Board." << endl;
        else cout << "Stop Getting Data from All Board." << endl;

        for(int i = 0;i<_NO_OF_VC; i++)
        {
            switch (type)
            {
            case 2:
                cout << "Data Type : Position & Velocity" << endl;
                _DEV_VC[i].CMD_Request_PosVel(enable);
                break;
            case 3:
                cout << "Data Type : Valve Position" << endl;
                _DEV_VC[i].CMD_Request_ValvePosnPWM(enable);
                break;
            case 4:
                cout << "Data Type : Actuator Pressure" << endl;
                _DEV_VC[i].CMD_Request_Pressure(enable);
                break;
            case 5:
                cout << "Data Type : Other Information" << endl;
                _DEV_VC[i].CMD_Request_OtherInfo(enable);
                break;
            case -1:
                cout << "Data Type : ALL DATA" << endl;
                _DEV_VC[i].CMD_Request_PosVel(enable);
                _DEV_VC[i].CMD_Request_ValvePosnPWM(enable);
                _DEV_VC[i].CMD_Request_Pressure(enable);
                _DEV_VC[i].CMD_Request_OtherInfo(enable);
                break;
            default:
                break;
            }
        }
    }
    else
    {
        if(enable) cout << "Request to Get Data from Board(" << id <<")"<< endl;
        else cout << "Stop Getting Data from Board(" << id <<")"<< endl;
        switch (type)
        {
        case 2:
            cout << "Data Type : Position & Velocity" << endl;
            _DEV_VC[id].CMD_Request_PosVel(enable);
            break;
        case 3:
            cout << "Data Type : Valve Position" << endl;
            _DEV_VC[id].CMD_Request_ValvePosnPWM(enable);
            break;
        case 4:
            cout << "Data Type : Actuator Pressure" << endl;
            _DEV_VC[id].CMD_Request_Pressure(enable);
            break;
        case 5:
            cout << "Data Type : Other Information" << endl;
            _DEV_VC[id].CMD_Request_OtherInfo(enable);
            break;
        case -1:
            cout << "Data Type : ALL DATA" << endl;
            _DEV_VC[id].CMD_Request_PosVel(enable);
            _DEV_VC[id].CMD_Request_ValvePosnPWM(enable);
            _DEV_VC[id].CMD_Request_Pressure(enable);
            _DEV_VC[id].CMD_Request_OtherInfo(enable);
            break;
        default:
            break;
        }
    }
}

void RBCMD_AskValveControllerParameters(int _BN) {
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_BOARDNUMBER);            usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_BOARDOPERATIONMODE);     usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_CANFREQ);                usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_CTRLMODE);               usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_JOINTENCDIR);            usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEINPUTDIR);          usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEENCDIR);            usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_BOARDINPUTVOLTAGE);      usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEOPERVOLTAGE);       usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VARIABLESUPPLY_ONOFF);   usleep(10*1000);
    argInt[0] = 0;
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_PIDGAIN, argInt);        usleep(10*1000);
    argInt[0] = 1;
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_PIDGAIN, argInt);        usleep(10*1000);
    argInt[0] = 2;
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_PIDGAIN, argInt);        usleep(10*1000);
    argInt[0] = 3;
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_PIDGAIN, argInt);        usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEDZ);                usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VELCOMP);                usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_COMPLIANCE);             usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_FEEDFORWARD);            usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_BULKMODULUS);            usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_CHAMBERVOL);             usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_PISTONAREA);             usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_SUPnRETPRES);            usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_JOINTENCLIMIT);          usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_PISTONSTROKE);           usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEPOSLIMIT);          usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_ENCPPP);                 usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_SENPPF);                 usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_SENPPP);                 usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_CONSTFRICTION);          usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEGAINPLUS);          usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_VALVEGAINMINUS);         usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_HOMEPOSOFFSET);          usleep(10*1000);
    _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_ASK_HOMEPOSOPENING);         usleep(10*1000);
}

void RBCMD_SetValveControllerParameters(int _BN, HCB_INFO H)
{
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.OPERATION_MODE != H.OPERATION_MODE
            || sharedSEN->ENCODER[_BN][0].HCB_Info.SENSING_MODE != H.SENSING_MODE
            || sharedSEN->ENCODER[_BN][0].HCB_Info.CURRENT_CONTROL_MODE != H.CURRENT_CONTROL_MODE)
    {
        cout << "[Parameters Change] Oper. Mode / Sense Mode / Current Ctrl. Mode" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.OPERATION_MODE << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.SENSING_MODE << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.CURRENT_CONTROL_MODE<< endl;
        cout << "New : " << H.OPERATION_MODE << " / " << H.SENSING_MODE << " / " << H.CURRENT_CONTROL_MODE << endl;
        argInt[0] = H.OPERATION_MODE;
        argInt[1] = H.SENSING_MODE;
        argInt[2] = H.CURRENT_CONTROL_MODE;
        argInt[3] = 0;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_BOARDOPERATIONMODE, argInt);
        usleep(50*1000);
    }

    if(sharedSEN->ENCODER[_BN][0].HCB_Info.CAN_FREQ != H.CAN_FREQ)
    {
        cout << "[Parameters Change] CAN Frequency" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.CAN_FREQ << endl;
        cout << "New : " << H.CAN_FREQ << endl;
        argInt[0] = H.CAN_FREQ;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_CANFREQ, argInt);
        usleep(50*1000);
    }

    //    _DEV_VC[_BN].CMD_ControlMode(H.CONTROL_MODE);
    //    usleep(50*1000);

    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_DIRECTION != H.JOINTENC_DIRECTION)
    {
        cout << "[Parameters Change] Joint Encoder Direction" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_DIRECTION << endl;
        cout << "New : " << H.JOINTENC_DIRECTION << endl;
        argInt[0] = H.JOINTENC_DIRECTION;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_JOINTENCDIR, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEINPUT_DIRECTION != H.VALVEINPUT_DIRECTION)
    {
        cout << "[Parameters Change] Valve Input Direction" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEINPUT_DIRECTION << endl;
        cout << "New : " << H.VALVEINPUT_DIRECTION << endl;
        argInt[0] = H.VALVEINPUT_DIRECTION;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_VALVEINPUTDIR, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEENC_DIRECTION != H.VALVEENC_DIRECTION)
    {
        cout << "[Parameters Change] Valve Encoder Direction" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEENC_DIRECTION << endl;
        cout << "New : " << H.VALVEENC_DIRECTION << endl;
        argInt[0] = H.VALVEENC_DIRECTION;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_VALVEENCDIR, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.BOARD_IN_VOLTAGE != H.BOARD_IN_VOLTAGE)
    {
        cout << "[Parameters Change] Board Input Voltage" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.BOARD_IN_VOLTAGE << endl;
        cout << "New : " << H.BOARD_IN_VOLTAGE << endl;
        argDouble[0] = H.BOARD_IN_VOLTAGE;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_BOARDINPUTVOLTAGE, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.BOARD_OPER_VOLTAGE != H.BOARD_OPER_VOLTAGE)
    {
        cout << "[Parameters Change] Board Operational Voltage" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.BOARD_OPER_VOLTAGE << endl;
        cout << "New : " << H.BOARD_OPER_VOLTAGE << endl;
        argDouble[0] = H.BOARD_OPER_VOLTAGE;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_VALVEOPERVOLTAGE, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF != H.VARIABLE_SUPPLYPRES_ONOFF)
    {
        cout << "[Parameters Change] Variable Supply Pressure On/Off" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF << endl;
        cout << "New : " << H.VARIABLE_SUPPLYPRES_ONOFF << endl;
        argInt[0] = H.VARIABLE_SUPPLYPRES_ONOFF;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_VARIABLESUPPLY_ONOFF, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KP != H.VALVE_P_KP
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KI != H.VALVE_P_KI
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KD != H.VALVE_P_KD)
    {
        cout << "[Parameters Change] Valve Control P / I / D Gain" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KP << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KI << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KD<< endl;
        cout << "New : " << H.VALVE_P_KP << " / " << H.VALVE_P_KI << " / " << H.VALVE_P_KD << endl;
        argDouble[0] = 0;
        argDouble[1] = H.VALVE_P_KP;
        argDouble[2] = H.VALVE_P_KI;
        argDouble[3] = H.VALVE_P_KD;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_PIDGAIN, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KP != H.JOINT_P_KP
            || sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KI != H.JOINT_P_KI
            || sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KD != H.JOINT_P_KD)
    {
        cout << "[Parameters Change] Joint Position Control P / I / D Gain" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KP << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KI << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KD<< endl;
        cout << "New : " << H.JOINT_P_KP << " / " << H.JOINT_P_KI << " / " << H.JOINT_P_KD << endl;
        argDouble[0] = 1;
        argDouble[1] = H.JOINT_P_KP;
        argDouble[2] = H.JOINT_P_KI;
        argDouble[3] = H.JOINT_P_KD;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_PIDGAIN, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KP != H.JOINT_F_KP
            || sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KI != H.JOINT_F_KI
            || sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KD != H.JOINT_F_KD)
    {
        cout << "[Parameters Change] Joint Force Control P / I / D Gain" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KP << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KI << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KD<< endl;
        cout << "New : " << H.JOINT_F_KP << " / " << H.JOINT_F_KI << " / " << H.JOINT_F_KD << endl;
        argDouble[0] = 2;
        argDouble[1] = H.JOINT_F_KP;
        argDouble[2] = H.JOINT_F_KI;
        argDouble[3] = H.JOINT_F_KD;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_PIDGAIN, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_SPRING != H.JOINT_SPRING
            || sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_DAMPER != H.JOINT_DAMPER)
    {
        cout << "[Parameters Change] Joint Spring / Damper Gain" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_SPRING << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_DAMPER<< endl;
        cout << "New : " << H.JOINT_SPRING << " / " << H.JOINT_DAMPER << endl;
        argDouble[0] = 3;
        argDouble[1] = H.JOINT_SPRING;
        argDouble[2] = H.JOINT_DAMPER;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_PIDGAIN, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_CENTER_POS != H.VALVE_CENTER_POS
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_DZ_PLUS != H.VALVE_DZ_PLUS
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_DZ_MINUS != H.VALVE_DZ_MINUS)
    {
        cout << "[Parameters Change] Valve Deadzone Center / Plus / Minus" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_CENTER_POS << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_DZ_PLUS << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_DZ_MINUS<< endl;
        cout << "New : " << H.VALVE_CENTER_POS << " / " << H.VALVE_DZ_PLUS << " / " << H.VALVE_DZ_MINUS << endl;
        argInt[0] = H.VALVE_CENTER_POS;
        argInt[1] = H.VALVE_DZ_PLUS;
        argInt[2] = H.VALVE_DZ_MINUS;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_VALVEDZ, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VEL_COMPENSATION_K != H.VEL_COMPENSATION_K)
    {
        cout << "[Parameters Change] Velocity Compensation" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VEL_COMPENSATION_K << endl;
        cout << "New : " << H.VEL_COMPENSATION_K << endl;
        argInt[0] = H.VEL_COMPENSATION_K;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_VELCOMP, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.ACTUATOR_COMPLIANCE_K != H.ACTUATOR_COMPLIANCE_K)
    {
        cout << "[Parameters Change] Actuator Compliance Gain" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.ACTUATOR_COMPLIANCE_K << endl;
        cout << "New : " << H.ACTUATOR_COMPLIANCE_K << endl;
        argInt[0] = H.ACTUATOR_COMPLIANCE_K;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_COMPLIANCE, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_FEEDFORWARD != H.VALVE_FEEDFORWARD)
    {
        cout << "[Parameters Change] Valve Feedforward" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_FEEDFORWARD << endl;
        cout << "New : " << H.VALVE_FEEDFORWARD << endl;
        argInt[0] = H.VALVE_FEEDFORWARD;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_FEEDFORWARD, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.BULK_MODULUS != H.BULK_MODULUS)
    {
        cout << "[Parameters Change] Bulk Modulus" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.BULK_MODULUS << endl;
        cout << "New : " << H.BULK_MODULUS << endl;
        argInt[0] = H.BULK_MODULUS;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_BULKMODULUS, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VOL_A != H.VOL_A
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VOL_B != H.VOL_B)
    {
        cout << "[Parameters Change] Chamber Volumn A / B" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VOL_A << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.VOL_B<< endl;
        cout << "New : " << H.VOL_A << " / " << H.VOL_B << endl;
        argInt[0] = H.VOL_A;
        argInt[1] = H.VOL_B;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_CHAMBERVOL, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_AREA_A != H.PIS_AREA_A
            || sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_AREA_B != H.PIS_AREA_B)
    {
        cout << "[Parameters Change] Piston Area A / B" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_AREA_A << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_AREA_B<< endl;
        cout << "New : " << H.PIS_AREA_A << " / " << H.PIS_AREA_B << endl;
        argInt[0] = H.PIS_AREA_A;
        argInt[1] = H.PIS_AREA_B;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_PISTONAREA, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.SUP_PRES != H.SUP_PRES)
    {
        cout << "[Parameters Change] Supply Pressure" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.SUP_PRES << endl;
        cout << "New : " << H.SUP_PRES << endl;
        argInt[0] = H.SUP_PRES;
        argInt[1] = 0;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_SUPnRETPRES, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_LIMIT_MINUS != H.JOINTENC_LIMIT_MINUS
            || sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_LIMIT_PLUS != H.JOINTENC_LIMIT_PLUS)
    {
        cout << "[Parameters Change] Joint Encoder Limit Minus / Plus" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_LIMIT_MINUS << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_LIMIT_PLUS<< endl;
        cout << "New : " << H.JOINTENC_LIMIT_MINUS << " / " << H.JOINTENC_LIMIT_PLUS << endl;
        argInt[0] = H.JOINTENC_LIMIT_MINUS;
        argInt[1] = H.JOINTENC_LIMIT_PLUS;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_JOINTENCLIMIT, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_STROKE != H.PIS_STROKE)
    {
        cout << "[Parameters Change] Piston Stroke" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_STROKE << endl;
        cout << "New : " << H.PIS_STROKE << endl;
        argInt[0] = H.PIS_STROKE;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_PISTONSTROKE, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEPOS_LIMIT_MINUS != H.VALVEPOS_LIMIT_MINUS
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEPOS_LIMIT_PLUS != H.VALVEPOS_LIMIT_PLUS)
    {
        cout << "[Parameters Change] Valve Position Limit Minus / Plus" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEPOS_LIMIT_MINUS << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEPOS_LIMIT_PLUS<< endl;
        cout << "New : " << H.VALVEPOS_LIMIT_MINUS << " / " << H.VALVEPOS_LIMIT_PLUS << endl;
        argInt[0] = H.VALVEPOS_LIMIT_MINUS;
        argInt[1] = H.VALVEPOS_LIMIT_PLUS;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_VALVEPOSLIMIT, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_PPP != H.JOINTENC_PPP)
    {
        cout << "[Parameters Change] Joint Encoder Pulse per Position" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_PPP << endl;
        cout << "New : " << H.JOINTENC_PPP << endl;
        argDouble[0] = H.JOINTENC_PPP;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_ENCPPP, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.FORCESEN_PPF != H.FORCESEN_PPF)
    {
        cout << "[Parameters Change] Force Sensor Pulse per Force" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.FORCESEN_PPF << endl;
        cout << "New : " << H.FORCESEN_PPF << endl;
        argDouble[0] = H.FORCESEN_PPF;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_SENPPF, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.PRESSEN_PPP_A != H.PRESSEN_PPP_A ||
            sharedSEN->ENCODER[_BN][0].HCB_Info.PRESSEN_PPP_B != H.PRESSEN_PPP_B)
    {
        cout << "[Parameters Change] Pressure Sensor Pulse per Pressure of A / B" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.PRESSEN_PPP_A << " / " << sharedSEN->ENCODER[_BN][0].HCB_Info.PRESSEN_PPP_B<< endl;
        cout << "New : " << H.PRESSEN_PPP_A << " / " << H.PRESSEN_PPP_B << endl;
        argDouble[0] = H.PRESSEN_PPP_A;
        argDouble[1] = H.PRESSEN_PPP_B;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_SENPPP, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.CONST_FRIC != H.CONST_FRIC)
    {
        cout << "[Parameters Change] Constant Friction" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.CONST_FRIC << endl;
        cout << "New : " << H.CONST_FRIC << endl;
        argDouble[0] = H.CONST_FRIC;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_CONSTFRICTION, argDouble);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.HOMEPOS_OFFSET != H.HOMEPOS_OFFSET)
    {
        cout << "[Parameters Change] Home Pose Offset" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.HOMEPOS_OFFSET << endl;
        cout << "New : " << H.HOMEPOS_OFFSET << endl;
        argInt[0] = H.HOMEPOS_OFFSET;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_HOMEPOSOFFSET, argInt);
        usleep(50*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.HOMEPOS_VALVE_OPENING != H.HOMEPOS_VALVE_OPENING)
    {
        cout << "[Parameters Change] Home Pose Valve Opening" << endl;
        cout << "Previous : " << sharedSEN->ENCODER[_BN][0].HCB_Info.HOMEPOS_VALVE_OPENING << endl;
        cout << "New : " << H.HOMEPOS_VALVE_OPENING << endl;
        argInt[0] = H.HOMEPOS_VALVE_OPENING;
        _DEV_VC[_BN].SendGeneralMSG(ValveController_GeneralMSG_CMD_HOMEPOSOPENING, argInt);
        usleep(50*1000);
    }
    _DEV_VC[_BN].Joints[0].HCB_Info = H;
    FILE_LOG(logSUCCESS) << "Board(" << _BN << "), Parameter Setting is Done!";
}

void RBCMD_ReadnSaveValveControllerParameters(void)
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "ValveControllerParameters/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();
    const char* fileBNO = "/BOARD";
    const char* filetype = ".txt";

    // Creating a directory
    if (mkdir(filedate, 0777) == -1) {
        cerr << " Creating Directory Error :  " << strerror(errno) << endl;
        return;
    }

    for(int i=0;i<_NO_OF_VC;i++) {
        if(_DEV_VC[i].ConnectionStatus) {
            FILE_LOG(logWARNING) << "Board(" << i << ") Loading Parameters...";
            RBCMD_AskValveControllerParameters(i);
            usleep(1000*1000);
            FILE_LOG(logSUCCESS) << "Board(" << i << ") Parameters are Loaded!";

            // Saving Parameters in Text File
            char filename[100];
            strcpy(filename, filedate);
            strcat(filename, fileBNO);
            strcat(filename, to_string(i).c_str());
            strcat(filename, filetype);

            FILE* fp;
            fp = fopen(filename, "w");
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.OPERATION_MODE);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.SENSING_MODE);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.CURRENT_CONTROL_MODE);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.CAN_FREQ);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_DIRECTION);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVEINPUT_DIRECTION);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVEENC_DIRECTION);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.BOARD_IN_VOLTAGE);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.BOARD_OPER_VOLTAGE);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KP);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KI);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KD);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KP);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KI);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KD);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KP);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KI);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KD);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_SPRING);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINT_DAMPER);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_CENTER_POS);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_DZ_PLUS);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_DZ_MINUS);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VEL_COMPENSATION_K);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.ACTUATOR_COMPLIANCE_K);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_FEEDFORWARD);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.BULK_MODULUS);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VOL_A);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VOL_B);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.PIS_AREA_A);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.PIS_AREA_B);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.SUP_PRES);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.RET_PRES);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_LIMIT_MINUS);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_LIMIT_PLUS);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.PIS_STROKE);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVEPOS_LIMIT_MINUS);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.VALVEPOS_LIMIT_PLUS);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_PPP);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.FORCESEN_PPF);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.PRESSEN_PPP_A);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.PRESSEN_PPP_B);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.CONST_FRIC);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[0]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[1]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[2]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[3]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[4]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[0]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[1]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[2]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[3]);
            fprintf(fp, "%lf\n", _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[4]);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.HOMEPOS_OFFSET);
            fprintf(fp, "%d\n", _DEV_VC[i].Joints[0].HCB_Info.HOMEPOS_VALVE_OPENING);
            fclose(fp);

            FILE_LOG(logSUCCESS) << "Board(" << i << ") Parameters are Saved!";
            FILE_LOG(logSUCCESS) << "Saved Filename : " << filename;
            usleep(500*1000);

        } else {
            FILE_LOG(logWARNING) << "BoardConnection (" << i << ") is not checked yet.";
        }
    }
}

void    RBCMD_LoadnSetValveControllerParameters(void)
{
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];
    string dir(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR);
    dir = dir.substr(0,41);
    const char* filedir = dir.c_str();
    const char* fileBNO = "/BOARD";
    const char* filetype = ".txt";

    if(_BN == -1) { // Set Parameters to All Joints
        for(int i=0;i<_NO_OF_VC;i++) {
            if(_DEV_VC[i].ConnectionStatus) {
                FILE_LOG(logWARNING) << "Board(" << i << ") Reading Parameters...";
                RBCMD_AskValveControllerParameters(i);
                usleep(1000*1000);
                FILE_LOG(logSUCCESS) << "Board(" << i << ") Reading Parameters is finished!";

                char filename[100];
                strcpy(filename, filedir);
                strcat(filename, fileBNO);
                strcat(filename, to_string(i).c_str());
                strcat(filename, filetype);

                HCB_INFO H_temp;
                FILE* pFile = fopen ( filename, "r" );
                if (pFile!=NULL) {
                    fscanf(pFile, "%d \n", &H_temp.OPERATION_MODE);
                    fscanf(pFile, "%d \n", &H_temp.SENSING_MODE);
                    fscanf(pFile, "%d \n", &H_temp.CURRENT_CONTROL_MODE);
                    fscanf(pFile, "%d \n", &H_temp.CAN_FREQ);
                    fscanf(pFile, "%d \n", &H_temp.JOINTENC_DIRECTION);
                    fscanf(pFile, "%d \n", &H_temp.VALVEINPUT_DIRECTION);
                    fscanf(pFile, "%d \n", &H_temp.VALVEENC_DIRECTION);
                    fscanf(pFile, "%lf \n", &H_temp.BOARD_IN_VOLTAGE);
                    fscanf(pFile, "%lf \n", &H_temp.BOARD_OPER_VOLTAGE);
                    fscanf(pFile, "%d \n", &H_temp.VARIABLE_SUPPLYPRES_ONOFF);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_P_KP);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_P_KI);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_P_KD);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_P_KP);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_P_KI);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_P_KD);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_F_KP);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_F_KI);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_F_KD);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_SPRING);
                    fscanf(pFile, "%lf \n", &H_temp.JOINT_DAMPER);
                    fscanf(pFile, "%d \n", &H_temp.VALVE_CENTER_POS);
                    fscanf(pFile, "%d \n", &H_temp.VALVE_DZ_PLUS);
                    fscanf(pFile, "%d \n", &H_temp.VALVE_DZ_MINUS);
                    fscanf(pFile, "%d \n", &H_temp.VEL_COMPENSATION_K);
                    fscanf(pFile, "%d \n", &H_temp.ACTUATOR_COMPLIANCE_K);
                    fscanf(pFile, "%d \n", &H_temp.VALVE_FEEDFORWARD);
                    fscanf(pFile, "%d \n", &H_temp.BULK_MODULUS);
                    fscanf(pFile, "%d \n", &H_temp.VOL_A);
                    fscanf(pFile, "%d \n", &H_temp.VOL_B);
                    fscanf(pFile, "%d \n", &H_temp.PIS_AREA_A);
                    fscanf(pFile, "%d \n", &H_temp.PIS_AREA_B);
                    fscanf(pFile, "%d \n", &H_temp.SUP_PRES);
                    fscanf(pFile, "%d \n", &H_temp.RET_PRES);
                    fscanf(pFile, "%d \n", &H_temp.JOINTENC_LIMIT_MINUS);
                    fscanf(pFile, "%d \n", &H_temp.JOINTENC_LIMIT_PLUS);
                    fscanf(pFile, "%d \n", &H_temp.PIS_STROKE);
                    fscanf(pFile, "%d \n", &H_temp.VALVEPOS_LIMIT_MINUS);
                    fscanf(pFile, "%d \n", &H_temp.VALVEPOS_LIMIT_PLUS);
                    fscanf(pFile, "%lf \n", &H_temp.JOINTENC_PPP);
                    fscanf(pFile, "%lf \n", &H_temp.FORCESEN_PPF);
                    fscanf(pFile, "%lf \n", &H_temp.PRESSEN_PPP_A);
                    fscanf(pFile, "%lf \n", &H_temp.PRESSEN_PPP_B);
                    fscanf(pFile, "%d \n", &H_temp.CONST_FRIC);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[0]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[1]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[2]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[3]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[4]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[0]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[1]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[2]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[3]);
                    fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[4]);
                    fscanf(pFile, "%d \n", &H_temp.HOMEPOS_OFFSET);
                    fscanf(pFile, "%d \n", &H_temp.HOMEPOS_VALVE_OPENING);
                } else {
                    FILE_LOG(logERROR) << "File error";
                    return;
                }
                fclose (pFile);
                RBCMD_SetValveControllerParameters(i, H_temp);
                FILE_LOG(logSUCCESS) << "Board(" << i << ") Parameters are Set!";
                usleep(500*1000);
            } else {
                FILE_LOG(logWARNING) << "BoardConnection(" << i << ") is not checked yet.";
            }
        }
    } else {  // Set Parameters to Single Selected Joint
        if(_DEV_VC[_BN].ConnectionStatus) {
            FILE_LOG(logWARNING) << "Board(" << _BN << ") Reading Parameters...";
            RBCMD_AskValveControllerParameters(_BN);
            usleep(1000*1000);
            FILE_LOG(logSUCCESS) << "Board(" << _BN << ") Reading Parameters is finished!";

            char filename[100];
            strcpy(filename, filedir);
            strcat(filename, fileBNO);
            strcat(filename, to_string(_BN).c_str());
            strcat(filename, filetype);

            HCB_INFO H_temp;
            FILE* pFile = fopen ( filename, "r" );
            if (pFile!=NULL) {
                fscanf(pFile, "%d \n", &H_temp.OPERATION_MODE);
                fscanf(pFile, "%d \n", &H_temp.SENSING_MODE);
                fscanf(pFile, "%d \n", &H_temp.CURRENT_CONTROL_MODE);
                fscanf(pFile, "%d \n", &H_temp.CAN_FREQ);
                fscanf(pFile, "%d \n", &H_temp.JOINTENC_DIRECTION);
                fscanf(pFile, "%d \n", &H_temp.VALVEINPUT_DIRECTION);
                fscanf(pFile, "%d \n", &H_temp.VALVEENC_DIRECTION);
                fscanf(pFile, "%lf \n", &H_temp.BOARD_IN_VOLTAGE);
                fscanf(pFile, "%lf \n", &H_temp.BOARD_OPER_VOLTAGE);
                fscanf(pFile, "%d \n", &H_temp.VARIABLE_SUPPLYPRES_ONOFF);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_P_KP);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_P_KI);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_P_KD);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_P_KP);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_P_KI);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_P_KD);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_F_KP);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_F_KI);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_F_KD);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_SPRING);
                fscanf(pFile, "%lf \n", &H_temp.JOINT_DAMPER);
                fscanf(pFile, "%d \n", &H_temp.VALVE_CENTER_POS);
                fscanf(pFile, "%d \n", &H_temp.VALVE_DZ_PLUS);
                fscanf(pFile, "%d \n", &H_temp.VALVE_DZ_MINUS);
                fscanf(pFile, "%d \n", &H_temp.VEL_COMPENSATION_K);
                fscanf(pFile, "%d \n", &H_temp.ACTUATOR_COMPLIANCE_K);
                fscanf(pFile, "%d \n", &H_temp.VALVE_FEEDFORWARD);
                fscanf(pFile, "%d \n", &H_temp.BULK_MODULUS);
                fscanf(pFile, "%d \n", &H_temp.VOL_A);
                fscanf(pFile, "%d \n", &H_temp.VOL_B);
                fscanf(pFile, "%d \n", &H_temp.PIS_AREA_A);
                fscanf(pFile, "%d \n", &H_temp.PIS_AREA_B);
                fscanf(pFile, "%d \n", &H_temp.SUP_PRES);
                fscanf(pFile, "%d \n", &H_temp.RET_PRES);
                fscanf(pFile, "%d \n", &H_temp.JOINTENC_LIMIT_MINUS);
                fscanf(pFile, "%d \n", &H_temp.JOINTENC_LIMIT_PLUS);
                fscanf(pFile, "%d \n", &H_temp.PIS_STROKE);
                fscanf(pFile, "%d \n", &H_temp.VALVEPOS_LIMIT_MINUS);
                fscanf(pFile, "%d \n", &H_temp.VALVEPOS_LIMIT_PLUS);
                fscanf(pFile, "%lf \n", &H_temp.JOINTENC_PPP);
                fscanf(pFile, "%lf \n", &H_temp.FORCESEN_PPF);
                fscanf(pFile, "%lf \n", &H_temp.PRESSEN_PPP_A);
                fscanf(pFile, "%lf \n", &H_temp.PRESSEN_PPP_B);
                fscanf(pFile, "%d \n", &H_temp.CONST_FRIC);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[0]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[1]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[2]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[3]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_PLUS[4]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[0]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[1]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[2]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[3]);
                fscanf(pFile, "%lf \n", &H_temp.VALVE_GAIN_MINUS[4]);
                fscanf(pFile, "%d \n", &H_temp.HOMEPOS_OFFSET);
                fscanf(pFile, "%d \n", &H_temp.HOMEPOS_VALVE_OPENING);
            } else {
                fputs ("File error",stderr);
                return;
            }
            fclose (pFile);
            RBCMD_SetValveControllerParameters(_BN, H_temp);
            FILE_LOG(logSUCCESS) << "Board(" << _BN << ") Parameters are Set!";
            usleep(500*1000);
        } else {
            FILE_LOG(logWARNING) << "BoardConnection(" << _BN << ") is not checked yet.";
        }
    }
}

// ============================================================================
// Board Operation Setting START

int  ValveController::CANCheck(void)
{
    ConnectionStatus = false; // For checking connection status, make it false once.
    SendGeneralMSG(ValveController_GeneralMSG_CANCHECK);
    usleep(100*1000); // Waiting for Checking ConnectionStatus
    if(ConnectionStatus){
        std::cout << ">>> MC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32mconnected.\033[0m[ch " << CAN_CHANNEL << "]\n";
        return true;
    } else {
        std::cout << ">>> MC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        return false;
    }
}

int  ValveController::CANChannel_Arrange(void){
    std::cout << " This Function is not used. \n";

//    std::cout << " ======== Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is checking channels... ========\n";

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
//                std::cout << ">>> Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32mconnected \033[0mto channel [" << idx << "]\n";
//                std::cout << " ========================================================\n\n";

//                CAN_CHANNEL = idx;
//                ConnectionStatus = true;
//                mb.status = RBCAN_NODATA;
//                return true;
//            }else{
//                std::cout << "\033[31m >>> [ Channel " << idx << " ] \is not connected... \033[0m " << std::endl;
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

int ValveController::InformationCheck(void){
    std::cout << " This Function is not used. \n";

//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;
//    mb.data[0] = 0; //Ask Board status(information)
//    mb.dlc = 1;
//    mb.id = ID_SEND_GENERAL;

//    if(canHandler->RBCAN_WriteData(mb)){
//        usleep(35*1000);
//        mb.channel = CAN_CHANNEL;
//        mb.id = ID_RCV_GENERAL;
//        canHandler->RBCAN_ReadData(&mb);
//        if(mb.status != RBCAN_NODATA && mb.data[0]==0 && mb.data[1]==BOARD_ID) {
//            Joints[0].HCB_Info.CAN_FREQ           = (int)((mb.data[2])|(mb.data[3]<<8));
//            Joints[0].HCB_Info.FET_ONOFF          = (mb.data[4]&0b10000000);
//            Joints[0].HCB_Info.BIGERROR_ONOFF     = (mb.data[4]&0b01000000);
//            Joints[0].HCB_Info.ENCERROR_ONOFF     = (mb.data[4]&0b00100000);
//            Joints[0].HCB_Info.CANERROR_ONOFF     = (mb.data[4]&0b00010000);
//            Joints[0].HCB_Info.HOMEERROR_ONOFF    = (mb.data[4]&0b00001000);
//            Joints[0].HCB_Info.PLIMITERROR_ONOFF  = (mb.data[4]&0b00000100);
//            Joints[0].HCB_Info.LOGICERROR_ONOFF   = (mb.data[4]&0b00000010);
//            Joints[0].HCB_Info.INPUTERROR_ONOFF   = (mb.data[4]&0b00000001);
//            switch(mb.data[5]){
//            case 0:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_NOACT; break;
//            case 1:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_OPENLOOP; break;
//            case 2:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_POS; break;
//            case 3:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_PWM; break;
//            case 4:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_VALVE; break;
//            case 5:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_LEARN; break;
//            case 6:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_PWM; break;
//            case 7:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_VALVE; break;
//            case 8:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_LEARN; break;
//            case 9:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_PWM_SINE_OPENLOOL; break;
//            case 20:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FORCE_NULL; break;
//            case 21:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_NULL; break;
//            case 22:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FINDHOME; break;
//            case 23:
//                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FLOWGAIN_TUNE; break;
//            default:
//                break;
//            }
//            if(mb.data[6]==0){
//                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SW;
//            } else {
//                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SH;
//            }
//            switch(mb.data[7]){
//            }

//            std::cout << "=================================== " << std::endl;
//            std::cout << "CAN FREQ : "  << Joints[0].HCB_Info.CAN_FREQ << std::endl;
//            std::cout << "CAN FET ERROR : " << Joints[0].HCB_Info.FET_ONOFF << std::endl;
//            std::cout << "CAN BIG ERROR : " << Joints[0].HCB_Info.BIGERROR_ONOFF << std::endl;
//            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.ENCERROR_ONOFF << std::endl;
//            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.CANERROR_ONOFF << std::endl;
//            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.HOMEERROR_ONOFF << std::endl;
//            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.PLIMITERROR_ONOFF << std::endl;
//            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.LOGICERROR_ONOFF << std::endl;
//            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.INPUTERROR_ONOFF << std::endl;
//            std::cout << "CAN Control Mode : " << Joints[0].HCB_Info.CONTROL_MODE << std::endl;
//            std::cout << "CAN Operation Mode : " << Joints[0].HCB_Info.OPERATION_MODE << std::endl;
//            std::cout << "=================================== " << std::endl;

//            ConnectionStatus = true;
//            mb.status = RBCAN_NODATA;
//            return true;
//        } else {
//            ConnectionStatus = false;
//            return false;
//        }
//    }
//    else return false;
}

int ValveController::CMD_ErrorClear(){
    if(ErrorFlag == true) {
        SendGeneralMSG(ValveController_GeneralMSG_CMD_ERRORCLEAR);
        ErrorFlag = false;
        std::cout << "Error Code Clear." << std::endl;
    } else {
//        std::cout << "Error Code is not found." << std::endl;
    }
}

enum JointControlMethod {
    ForceControl = 1,
    PositionControl = 3,
};

void ValveController::CMD_ControlMode(int mode){
    int argInt[4] = {0,};
    argInt[0] = mode;
    SendGeneralMSG(ValveController_GeneralMSG_CMD_CTRLMODE, argInt);
    Joints[0].HCB_Info.CONTROL_MODE = argInt[0];
}

int ValveController::CMD_Request_PosVel(bool OnOff){
    Joints[0].HCB_Info.REQUEST_ONOFF_PosVel = OnOff;
    int argInt[4] = {0,};
    argInt[0] = OnOff;
    argInt[1] = 0; // PosVel Request
    SendGeneralMSG(ValveController_GeneralMSG_CMD_DATAREQUEST_ONOFF, argInt);
}

int ValveController::CMD_Request_ValvePosnPWM(bool OnOff){
    Joints[0].HCB_Info.REQUEST_ONOFF_ValvePosnPWM = OnOff;
    int argInt[4] = {0,};
    argInt[0] = OnOff;
    argInt[1] = 1; // ValvePos Request
    SendGeneralMSG(ValveController_GeneralMSG_CMD_DATAREQUEST_ONOFF, argInt);
}

int ValveController::CMD_Request_Pressure(bool OnOff){
    Joints[0].HCB_Info.REQUEST_ONOFF_Pressure = OnOff;
    int argInt[4] = {0,};
    argInt[0] = OnOff;
    argInt[1] = 2; // Pressure Request
    SendGeneralMSG(ValveController_GeneralMSG_CMD_DATAREQUEST_ONOFF, argInt);
}

int ValveController::CMD_Request_OtherInfo(bool OnOff){
    Joints[0].HCB_Info.REQUEST_ONOFF_OtherInfo = OnOff;
    int argInt[4] = {0,};
    argInt[0] = OnOff;
    argInt[1] = 3; // Data for Debugging Request
    SendGeneralMSG(ValveController_GeneralMSG_CMD_DATAREQUEST_ONOFF, argInt);
}


int ValveController::ResetReference_PosVel(bool PsVar_OnOff)
{
    int16_t ref_pos = 0;
    int16_t ref_vel = 0;
    int16_t ref_force_pulse = 0;
    int16_t ref_pressure = 100;

    Joints[0].HCB_Ref.ReferencePosition = ref_pos;
    Joints[0].HCB_Ref.ReferenceVelocity = ref_vel;
    Joints[0].HCB_Ref.ReferenceForceTorque = ref_force_pulse;
    Joints[0].HCB_Ref.ReferencePumpPressure = ref_pressure;

    ST_CAN mb;
    mb.id = ID_SEND_POSVEL; // Position and Velocity Reference
    if(PsVar_OnOff) {
        mb.dlc = 8;
        mb.data[0] = (unsigned char)(ref_pos & 0xFF);
        mb.data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
        mb.data[2] = (unsigned char)(ref_vel & 0xFF);
        mb.data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
        mb.data[4] = (unsigned char)(ref_force_pulse & 0xFF);
        mb.data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
        mb.data[6] = (unsigned char)(ref_pressure & 0xFF);
        mb.data[7] = (unsigned char)((ref_pressure>>8) & 0xFF);
    } else {
        mb.dlc = 6;
        mb.data[0] = (unsigned char)(ref_pos & 0xFF);
        mb.data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
        mb.data[2] = (unsigned char)(ref_vel & 0xFF);
        mb.data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
        mb.data[4] = (unsigned char)(ref_force_pulse & 0xFF);
        mb.data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
    }
    rb_can::write_general_msg(mb, CAN_CHANNEL);
}

