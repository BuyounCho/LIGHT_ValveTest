#include "RBMotorControllerCOCOA.h"
#include <iostream>
using namespace std;

//extern int     __IS_EXF_R;     // extra finger right
//extern int     __IS_EXF_L;     // extra finger left
//extern float   EXF_R_Modifier[5];
//extern float   EXF_L_Modifier[5];

RBMotorControllerCOCOA::RBMotorControllerCOCOA()
{
    for(int i=0; i<MAX_JOINT; i++){
        MoveJoints[i].MoveFlag = false;
    }
}

void RBMotorControllerCOCOA::RBJoint_SetMoveJoint(int ch, float angle, float timeMs, int mode){
    if(MoveJoints[ch].MoveFlag == true)
    {
        FILE_LOG(logWARNING) << "It's working now..[BNO: " << BOARD_ID << "]";
        return;
    }
    MoveJoints[ch].RefAngleInitial = MoveJoints[ch].RefAngleCurrent;
    if(mode == 0) // abs
    {
        MoveJoints[ch].RefAngleToGo = angle;
        MoveJoints[ch].RefAngleDelta = MoveJoints[ch].RefAngleToGo - MoveJoints[ch].RefAngleInitial;
    }
    else // rel
    {
        MoveJoints[ch].RefAngleToGo = MoveJoints[ch].RefAngleInitial + angle;
        MoveJoints[ch].RefAngleDelta = angle;
    }

    MoveJoints[ch].GoalTimeCount = (ulong)(timeMs/(double)RT_TIMER_PERIOD_MS);
    MoveJoints[ch].CurrentTimeCount = 0;
    MoveJoints[ch].MoveFlag = true;
}
void RBMotorControllerCOCOA::RBJoint_MoveJoint(int ch){
    if(MoveJoints[ch].MoveFlag)
    {
        MoveJoints[ch].CurrentTimeCount++;
        if(MoveJoints[ch].GoalTimeCount <= MoveJoints[ch].CurrentTimeCount)
        {
            MoveJoints[ch].GoalTimeCount = MoveJoints[ch].CurrentTimeCount = 0;
            MoveJoints[ch].RefAngleCurrent = MoveJoints[ch].RefAngleToGo;
            MoveJoints[ch].MoveFlag = false;
        }
        else
        {
            MoveJoints[ch].RefAngleCurrent = MoveJoints[ch].RefAngleInitial + MoveJoints[ch].RefAngleDelta*0.5*
                    (1.0f-cos(RBCORE_PI/(double)MoveJoints[ch].GoalTimeCount*(double)MoveJoints[ch].CurrentTimeCount));
        }
    }
}
void RBMotorControllerCOCOA::RBBoard_MoveJoint(){
    for(int i=0; i<1; i++){
        RBJoint_MoveJoint(i);
    }
}

void RBMotorControllerCOCOA::RBMC_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_ENC);
    canHandler->RBCAN_AddMailBox(ID_RCV_GENERAL);
    canHandler->RBCAN_AddMailBox(ID_RCV_CURRENT);
    canHandler->RBCAN_AddMailBox(ID_RCV_PWM);
//    canHandler->RBCAN_AddMailBox(ID_RCV_PWM+100); // torque sensor data
    canHandler->RBCAN_AddMailBox(ID_RCV_TORQUE); // torque sensor data
    std::cout<<"ttst "<<ID_RCV_ENC <<" "<<ID_RCV_GENERAL <<" "<<ID_RCV_CURRENT <<" "<<ID_RCV_PWM <<" "<<ID_RCV_TORQUE <<" "<<std::endl;
}
void RBMotorControllerCOCOA::RBBoard_GetDBData(DB_MC db){
    BOARD_NAME = db.BOARD_NAME;
    BOARD_ID = db.BOARD_ID;
    CAN_CHANNEL = db.CAN_CHANNEL;
    ID_SEND_GENERAL = db.ID_SEND_GENERAL;
    ID_SEND_REF = db.ID_SEND_REF;
    ID_SEND_REF_CURRENT = db.ID_SEND_REF_CURRENT;
    ID_SEND_PWM = db.ID_SEND_PWM;


    ID_RCV_GENERAL = db.ID_RCV_GENERAL;
    ID_RCV_ENC = db.ID_RCV_ENC;
    ID_RCV_CURRENT = db.ID_RCV_CURRENT;
    ID_RCV_PWM = db.ID_RCV_PWM;
    ID_RCV_TORQUE = db.ID_RCV_TORQUE; // Seungwoo added

    Joints[0].PPR = db.JOINTS[0].PPR;

}
void RBMotorControllerCOCOA::RBBoard_ReferenceOutEnable(bool _refEnable){
    ReferenceOutEnable = _refEnable;
}
int  RBMotorControllerCOCOA::RBBoard_CANCheck(int _canr){
    //Current_nulling and ask Board control gain
    //Current_nulling
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 8; //Board status information
    mb.dlc = 1;
//    mb.data[0] = 0; // Control mode setting
//    mb.data[1] = 2; // Current nulling
//    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
//    canHandler->RBCAN_WriteData(mb);
    usleep(15*1000);

    /*
    //Board control gain(Current control)
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 4;
    mb.data[1] = 1;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    */

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            std::cout << ">>> RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            CANRate = 500;
            Version = 0.1;
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}


int RBMotorControllerCOCOA::RBBoard_RequestEncoder(int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 1;
    mb.data[1] = mode;      // 1-on/0-off
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBBoard_RequestCurrent(int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 3;
    mb.data[1] = mode; // 1-on/0-off
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBBoard_RequestPWM(int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x07;
    mb.data[1] = mode; // 1-on/0-off
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
//    return true;
}
int RBMotorControllerCOCOA::RBJoint_ResetEncoder(int ch){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 2;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}


int RBMotorControllerCOCOA::RBJoint_EnableFeedbackControl_POSITION(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 1; // Position servo
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFeedbackControl_CURRENT(int ch, int enable){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 3; // Current servo
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFeedbackControl_CURRENTPOSITION(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 14; // Six-Step Current based Position servo
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    cout <<"hell yeah" << endl;
    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFeedbackControl_FOC_POSITION(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 4; // FOC-Position servo
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFeedbackControl_FOC_CURRENT(int ch, int enable){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 5; // FOC-Current servo
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFOCPWMControl(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 13; // FOC PWM
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnablePWMControl(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 7; // Manual PWM
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFETDriver(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 6;
    mb.data[1] = enable;        // 1-enable, 0-disable
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}


int RBMotorControllerCOCOA::RBBoard_SendReference(void){
    int i;
    int ref[2];

    if(ReferenceOutEnable == false) return false;

    if(1)
    {
        if(Joints[0].ControlMode == CONTROL_MODE_POS) // Position
        {
            Joints[0].RefPosOld = Joints[0].RefPos;
            Joints[0].RefPos = Joints[0].Reference;
            Joints[0].RefVel = (Joints[0].RefPos-Joints[0].RefPosOld)/(RT_TIMER_PERIOD_MS/1000.0);
            ref[0] = (int)(Joints[0].RefPos*Joints[0].PPR);

            return RBBoard_SendReferencePosition(ref[0]);
        }
        else if(Joints[0].ControlMode == CONTROL_MODE_CUR) // Current
        {
            return RBBoard_SendReferenceCurrent(Joints[0].RefCurrent);
        }
        else if(Joints[0].ControlMode == CONTROL_MODE_FOC_POS) // FOC-Position
        {
            Joints[0].RefPosOld = Joints[0].RefPos;
            Joints[0].RefPos = Joints[0].Reference;
            Joints[0].RefVel = (Joints[0].RefPos-Joints[0].RefPosOld)/(RT_TIMER_PERIOD_MS/1000.0);
            ref[0] = (int)(Joints[0].RefPos*Joints[0].PPR);

            return RBBoard_SendReferenceFOCPosition(ref[0]);
        }
        else if(Joints[0].ControlMode == CONTROL_MODE_FOC_CUR) // FOC-Current
        {
            return RBBoard_SendReferenceFOCCurrent(Joints[0].RefQCurrent, Joints[0].RefDCurrent);
        }
        else if(Joints[0].ControlMode == CONTROL_MODE_PWM)
        {
            return RBBoard_SendReferencePWM(Joints[0].RefPWM);
        }
        else if(Joints[0].ControlMode == CONTROL_MODE_CUR_POS) // CUR-Position //GG added
        {
            Joints[0].RefPosOld = Joints[0].RefPos;
            Joints[0].RefPos = Joints[0].Reference;
            Joints[0].RefVel = (Joints[0].RefPos-Joints[0].RefPosOld)/(RT_TIMER_PERIOD_MS/1000.0);
            ref[0] = (int)(Joints[0].RefPos*Joints[0].PPR);

            return RBBoard_SendReferencePosition(ref[0]);
        }
        else if(Joints[0].ControlMode == CONTROL_MODE_CUR_POS_FF) // CUR-Position //GG added
        {
            Joints[0].RefPosOld = Joints[0].RefPos;
            Joints[0].RefPos = Joints[0].Reference;
            Joints[0].RefVel = (Joints[0].RefPos-Joints[0].RefPosOld)/(RT_TIMER_PERIOD_MS/1000.0);
            ref[0] = (int)(Joints[0].RefPos*Joints[0].PPR);

            return RBBoard_SendReferencePositionPlusFF(ref[0],Joints[0].RefCurrent);
        }
    }
    return false;
}
int RBMotorControllerCOCOA::RBBoard_SendReferencePositionPlusFF(int ref1, double Current){

    int16_t CurRef = 100.*Current;//to 100*current;
    if(CurRef>40*100) CurRef = 40*100;
    if(CurRef<-40*100) CurRef = -40*100;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_REF; // Position (or Velocity) Reference
    mb.dlc = 7;
    mb.data[0] = (unsigned char)(ref1 & 0x000000FF);
    mb.data[1] = (unsigned char)((ref1>>8) & 0x000000FF);
    mb.data[2] = (unsigned char)((ref1>>16) & 0x000000FF);
    mb.data[3] = (unsigned char)((ref1>>24) & 0x000000FF);
    memcpy(&(mb.data[4]),&CurRef,2);
    mb.data[6] = 1;//auto check
    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBBoard_SendReferencePosition(int ref1){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_REF; // Position (or Velocity) Reference
    mb.dlc = 5;
    mb.data[0] = (unsigned char)(ref1 & 0x000000FF);
    mb.data[1] = (unsigned char)((ref1>>8) & 0x000000FF);
    mb.data[2] = (unsigned char)((ref1>>16) & 0x000000FF);
    mb.data[3] = (unsigned char)((ref1>>24) & 0x000000FF);
    mb.data[4] = 1;//auto check
    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBBoard_SendReferenceCurrent(double Current){
    int16_t CurRef = 100*Current;//to 100*current;
//    if(CurRef>15*100) CurRef = 15*100;
//    if(CurRef<-15*100) CurRef = -15*100;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_REF_CURRENT;
    mb.dlc = 3;
//    mb.data[0] = (unsigned char)(CurRef & (int16_t)0x00FF);
//    mb.data[1] = (unsigned char)((CurRef>>8) & (int16_t)0x00FF);
    memcpy(mb.data,&CurRef,2);
    mb.data[2] = 1;//auto check
//    std::cout<<(int)mb.data[0]<<" "<<(int)mb.data[1]<<std::endl;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBBoard_SendReferenceFOCPosition(int ref1){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_REF; // Position (or Velocity) Reference
    mb.dlc = 5;
    mb.data[0] = (unsigned char)(ref1 & 0x000000FF);
    mb.data[1] = (unsigned char)((ref1>>8) & 0x000000FF);
    mb.data[2] = (unsigned char)((ref1>>16) & 0x000000FF);
    mb.data[3] = (unsigned char)((ref1>>24) & 0x000000FF);
    mb.data[4] = 1;//auto check
    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBBoard_SendReferenceFOCCurrent(double Q_Current, double D_Current){
    int16_t Q_CurRef = 100*Q_Current;//to 100*Q_current;
    int16_t D_CurRef = 100*D_Current;//to 100*D_current;

//    if(Q_CurRef>15*100) Q_CurRef = 50*100;
//    if(Q_CurRef<-15*100) Q_CurRef = -50*100;
//    if(D_CurRef>15*100) D_CurRef = 50*100;
//    if(D_CurRef<-15*100) D_CurRef = -50*100;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_REF_CURRENT;
    mb.dlc = 5;
//    mb.data[0] = (unsigned char)(Q_CurRef & (int16_t)0x00FF);
//    mb.data[1] = (unsigned char)((Q_CurRef>>8) & (int16_t)0x00FF);
//    mb.data[2] = (unsigned char)(D_CurRef & (int16_t)0x00FF);
//    mb.data[3] = (unsigned char)((D_CurRef>>8) & (int16_t)0x00FF);

    memcpy(&(mb.data[0]),&Q_CurRef,2);
    memcpy(&(mb.data[2]),&D_CurRef,2);
    mb.data[4] = 1;//auto check
//    std::cout<<(int)mb.data[0]<<" "<<(int)mb.data[1]<<std::endl;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBBoard_SendReferencePWM(double ref1){

    RBCAN_MB mb;
    int16_t pwmref = ref1*10;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_PWM;
    mb.dlc = 3;
    memcpy(mb.data,&pwmref,2);
    mb.data[2] = 1;//auto check
//    mb.printMB();
//    FILE_LOG(logWARNING) << BOARD_ID << ", " <<ID_SEND_PWM<<" , "<<pwmref;
    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBBoard_Internal_Position_GOTO(int ref1){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_GENERAL;
    mb.dlc = 5;
    mb.data[0] = 24;
    mb.data[1] = (unsigned char)(ref1 & 0x000000FF);
    mb.data[2] = (unsigned char)((ref1>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)((ref1>>16) & 0x000000FF);
    mb.data[4] = (unsigned char)((ref1>>24) & 0x000000FF);

    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBBoard_SendReferenceCurrentPosition(int ref1){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_REF; // Position (or Velocity) Reference
    mb.dlc = 5;
    mb.data[0] = (unsigned char)(ref1 & 0x000000FF);
    mb.data[1] = (unsigned char)((ref1>>8) & 0x000000FF);
    mb.data[2] = (unsigned char)((ref1>>16) & 0x000000FF);
    mb.data[3] = (unsigned char)((ref1>>24) & 0x000000FF);
    mb.data[4] = 1;//auto check
    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBMotorControllerCOCOA::RBBoard_ReadEncoderData(void){
    RBCAN_MB mb;
    double tempDouble;

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_ENC;
    canHandler->RBCAN_ReadData(&mb);

    if(mb.status != RBCAN_NODATA){
        if(1){
            tempDouble = Joints[0].CurrentPosition;
            Joints[0].EncoderValue = (int)((mb.data[0])|(mb.data[1]<<8)|(mb.data[2]<<16)|(mb.data[3]<<24));
            Joints[0].CurrentPosition = (double)Joints[0].EncoderValue/Joints[0].PPR;
//            Joints[0].CurrentPosition = (double)Joints[0].EncoderValue/1000.0;

            tempDouble = (int)((mb.data[4])|(mb.data[5]<<8)|(mb.data[6]<<16)|(mb.data[7]<<24));
            Joints[0].CurrentVelocity = tempDouble/Joints[0].PPR;
//            Joints[0].CurrentVelocity = tempDouble/1000.0;
        }

        //FILE_LOG(logWARNING) << BOARD_ID << ", " << Joints[0].CurrentPosition << ", " << Joints[1].CurrentPosition;
        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}
int RBMotorControllerCOCOA::RBBoard_ReadTorqueSensorData(void){
    RBCAN_MB mb;

    mb.channel = CAN_CHANNEL;
//    mb.id = ID_RCV_PWM+100;
    mb.id = ID_RCV_TORQUE; // Seungwoo added
    canHandler->RBCAN_ReadData(&mb);

    if(mb.status != RBCAN_NODATA){
//        if(1){

//            Joints[0].Cocoa_Data.TORQUE_SENSOR1 = (int)((mb.data[0])|(mb.data[1]<<8)|(mb.data[2]<<16)|(mb.data[3]<<24));
//            Joints[0].Cocoa_Data.TORQUE_SENSOR2 = (int)((mb.data[4])|(mb.data[5]<<8)|(mb.data[6]<<16)|(mb.data[7]<<24));

//        }
        int adc_torque_1 = (int16_t)((mb.data[0])|(mb.data[1]<<8)); // Seungwoo added
//        int adc_torque_1 = (int16_t)((mb.data[0])|(mb.data[1]<<8)|(mb.data[2]<<16)); // Seungwoo added
        int adc_torque_2 = (int16_t)((mb.data[2])|(mb.data[3]<<8)); // Seungwoo added
//        int adc_torque_2 = (int16_t)((mb.data[3])|(mb.data[4]<<8)|(mb.data[5]<<16)); // Seungwoo added
//        int temp_1 = (int16_t)((mb.data[0])|(mb.data[1]<<8)); // Seungwoo added
//        int temp_2 = (int16_t)((mb.data[2])|(mb.data[3]<<8)); // Seungwoo added
//        double adc_torque_1 = 0.01*temp_1;
//        double adc_torque_2 = 0.01*temp_2;
//        int uart_torque_1 = (int16_t)((mb.data[4])|(mb.data[5]<<8)); // Seungwoo added
//        int uart_torque_2 = (int16_t)((mb.data[6])|(mb.data[7]<<8)); // Seungwoo added
//        Joints[0].CurrentTorque = 0.0001*torque; // Seungwoo added
        Joints[0].CurrentTorque1 = 0.01*adc_torque_1; // Seungwoo added
        Joints[0].CurrentTorque2 = 0.01*adc_torque_2; // Seungwoo added
//        Joints[0].CurrentTorque1 = ceilf(adc_torque_1*10.0)/10.0; // Seungwoo added
//        Joints[0].CurrentTorque2 = ceilf(adc_torque_2*100.0)/100.0; // Seungwoo added
//        Joints[0].UART_Torque1 = 0.01*uart_torque_1; // Seungwoo added
//        Joints[0].UART_Torque2 = 0.01*uart_torque_2; // Seungwoo added
        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}
int RBMotorControllerCOCOA::RBBoard_ReadCurrentData(void){
    RBCAN_MB mb;

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_CURRENT;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        int tta = (int16_t)((mb.data[2])|(mb.data[3]<<8));
        int ttb = (int16_t)((mb.data[4])|(mb.data[5]<<8));
        int ttc = (int16_t)((mb.data[6])|(mb.data[7]<<8));

        Joints[0].CurrentCurrenta = 0.01*tta;//|(mb.data[2]<<16)|(mb.data[3]<<24));
        Joints[0].CurrentCurrentb = 0.01*ttb;//|(mb.data[2]<<16)|(mb.data[3]<<24));
        Joints[0].CurrentCurrentc = 0.01*ttc;//|(mb.data[2]<<16)|(mb.data[3]<<24));

//        if(tt>32767){tt = tt-65536;}
        int tt = (int16_t)((mb.data[0])|(mb.data[1]<<8));
        Joints[0].CurrentCurrent = 0.01*tt;//|(mb.data[2]<<16)|(mb.data[3]<<24));
        mb.status = RBCAN_NODATA;

//        cout << "Current = " << 0.01*tt << endl;
//        cout << "Current = " << tt << endl;

        return true;
    }
    return false;
}
int RBMotorControllerCOCOA::RBBoard_ReadPWMData(void){
    RBCAN_MB mb;

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_PWM;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        Joints[0].CurrentPWM = (int16_t)((mb.data[0])|(mb.data[1]<<8));//|(mb.data[2]<<16)|(mb.data[3]<<24));
        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}

//COCOA_SPECIFIC?
void    RBMotorControllerCOCOA::RBJoint_FINDHOME(int type)
{
//    if(type!=0 && type!=1)
//    {
//        std::cout<<"type ERROR! should be 0 or 1"<<std::endl;
//        std::cout<<"0 : limit-switch mode, 1 : current-limit mode"<<std::endl;
//        return;
//    }
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 16;
    mb.data[1] = type; // 0: limit-switch mode, 1: current-limit mode

    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
//    return;
}
void    RBMotorControllerCOCOA::RBJoint_Request_Gain(int type)
{
    if(type!=0 && type!=1 && type!=2 && type!=3)
    {
        std::cout<<"type ERROR! should be 0 or 1 or 2 or 3"<<std::endl;
        return;
    }
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 4;
    mb.data[1] = type;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA && mb.data[0]==1){
        if(mb.data[1]==0) // Position control gain
        {
            uint16_t uI;
            memcpy(&(uI),&(mb.data[2]),2);
            Joints[0].Cocoa_Data.P_KP = uI*0.1;
            memcpy(&(uI),&(mb.data[4]),2);
            Joints[0].Cocoa_Data.P_KI = uI*0.1;
            memcpy(&(uI),&(mb.data[6]),2);
            Joints[0].Cocoa_Data.P_KD = uI*0.1;
        }
        if(mb.data[1]==1) // Current control gain
        {
            uint16_t uI;
            memcpy(&(uI),&(mb.data[2]),2);
            Joints[0].Cocoa_Data.C_KP = uI*0.1;
            memcpy(&(uI),&(mb.data[4]),2);
            Joints[0].Cocoa_Data.C_KI = uI*0.1;
            memcpy(&(uI),&(mb.data[6]),2);
            Joints[0].Cocoa_Data.C_KD = uI*0.1;
        }
        if(mb.data[1]==2) // Foc-position control gain
        {
            uint16_t uI;
            memcpy(&(uI),&(mb.data[2]),2);
            Joints[0].Cocoa_Data.FOC_P_KP = uI*0.1;
            memcpy(&(uI),&(mb.data[4]),2);
            Joints[0].Cocoa_Data.FOC_P_KI = uI*0.1;
            memcpy(&(uI),&(mb.data[6]),2);
            Joints[0].Cocoa_Data.FOC_P_KD = uI*0.1;
        }
        if(mb.data[1]==3) // Foc-current control gain
        {
            uint16_t uI;
            memcpy(&(uI),&(mb.data[2]),2);
            Joints[0].Cocoa_Data.FOC_C_KP = uI*0.1;
            memcpy(&(uI),&(mb.data[4]),2);
            Joints[0].Cocoa_Data.FOC_C_KI = uI*0.1;
            memcpy(&(uI),&(mb.data[6]),2);
            Joints[0].Cocoa_Data.FOC_C_KD = uI*0.1;
        }

        mb.status = RBCAN_NODATA;
        //mb.printMB();
    }
    else\
    {
        std::cout << ">>> REQGAIN_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
    return;

}
void    RBMotorControllerCOCOA::RBJoint_Request_Status()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 8;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    std::cout << ">>> REQSTATUS: mbdata" << ((int)(mb.data[0]))<<std::endl;
    if(mb.status != RBCAN_NODATA&&mb.data[0]==0){
        uint16_t cf;
        memcpy(&(cf),&(mb.data[2]),2);
        CANRate = cf;
        if(mb.data[4]&0b10000000){Joints[0].Cocoa_Data.FET_ONOFF = true;}
        else{Joints[0].Cocoa_Data.FET_ONOFF = false;}
        if(mb.data[4]&0b01000000){Joints[0].Cocoa_Data.BIGERROR_ONOFF = true;}
        else{Joints[0].Cocoa_Data.BIGERROR_ONOFF = false;}
        if(mb.data[4]&0b00100000){Joints[0].Cocoa_Data.ENCERROR_ONOFF = true;}
        else{Joints[0].Cocoa_Data.ENCERROR_ONOFF = false;}
        if(mb.data[4]&0b00010000){Joints[0].Cocoa_Data.CANERROR_ONOFF = true;}
        else{Joints[0].Cocoa_Data.CANERROR_ONOFF = false;}
        if(mb.data[4]&0b00001000){Joints[0].Cocoa_Data.HOMEERROR_ONOFF = true;}
        else{Joints[0].Cocoa_Data.HOMEERROR_ONOFF = false;}
        if(mb.data[4]&0b00000100){Joints[0].Cocoa_Data.PLIMITERROR_ONOFF = true;}
        else{Joints[0].Cocoa_Data.PLIMITERROR_ONOFF = false;}
        if(mb.data[4]&0b00000010){Joints[0].Cocoa_Data.LOGICERROR_ONOFF = true;}
        else{Joints[0].Cocoa_Data.HOMEERROR_ONOFF = false;}
        if(mb.data[4]&0b00000001){Joints[0].Cocoa_Data.INPUTERROR_ONOFF = true;}
        else{Joints[0].Cocoa_Data.HOMEERROR_ONOFF = false;}

        Joints[0].Cocoa_Data.BOARD_ACT = mb.data[5];
        Joints[0].Cocoa_Data.HOME_STATE = mb.data[6];

        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQSTATUS_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_FINDHOME_PARAMS()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 13;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==4){
        uint16_t cf;
        memcpy(&(cf),&(mb.data[1]),2);
        Joints[0].Cocoa_Data.FINDHOME_SEARCH_VEL =cf;
        if(mb.data[3]==1){Joints[0].Cocoa_Data.FINDHOME_DIRECTION = true;}
        else{Joints[0].Cocoa_Data.FINDHOME_DIRECTION = false;}
        memcpy(&(cf),&(mb.data[4]),2);
        Joints[0].Cocoa_Data.FINDHOME_OFFSET =cf;
        if(mb.data[6]==1){Joints[0].Cocoa_Data.FINDHOME_OFFSET_DIRECTION = true;}
        else{Joints[0].Cocoa_Data.FINDHOME_OFFSET_DIRECTION = false;}

        mb.status = RBCAN_NODATA;
    }
    else\
    {
        std::cout << ">>>REQFINDHOMEPARAM_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_PWM_RATE_LIMIT()\
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 9;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==2){
        uint16_t cf;
        memcpy(&(cf),&(mb.data[1]),2);
        Joints[0].Cocoa_Data.PWM_RATE_LIMIT = cf/100.0;

        mb.status = RBCAN_NODATA;
    }
    else\
    {
        std::cout << ">>> REQPWMLIM_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_Current_LIM()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 17;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==5){
        Joints[0].Cocoa_Data.CURRENT_LIMIT = mb.data[1];
        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQCURRLIM_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_Motor_Direction()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 19;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==6){

        if(mb.data[1]==1){Joints[0].Cocoa_Data.MOTOR_DIRECTION = true;}
        else{Joints[0].Cocoa_Data.MOTOR_DIRECTION = false;}

        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQMD_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}





void    RBMotorControllerCOCOA::RBJoint_Set_Gain(int type, double Kp, double Ki, double Kd)
{
    if(type!=0 && type!=1 && type!=2 && type!=3)
    {
        std::cout<<"type ERROR! should be 0 or 1 or 2 or 3"<<std::endl;
        return;
    }
    uint16_t p = Kp*10;
    uint16_t i = Ki*10;
    uint16_t d = Kd*10;



    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 5;
    mb.data[1] = type;
    memcpy(&(mb.data[2]),&p,2);
    memcpy(&(mb.data[4]),&i,2);
    memcpy(&(mb.data[6]),&d,2);
    mb.dlc = 8;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(50*1000);

    RBJoint_Request_Gain(type);

}
void    RBMotorControllerCOCOA::RBJoint_Set_Gain_Auto_Tune(int type)
{
    if(type!=0 && type!=8 && type!=9 && type!=10)
    {
        std::cout<<"type ERROR! should be 0 or 8 or 9 or 10"<<std::endl;
        return;
    }

    if(type==0)
    {
        std::cout<<"No Auto Tune for Current Control !"<<std::endl;
    }

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    mb.data[1] = type;
//    mb.data[2] = SLOW_GAIN_STEP_NUM;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(50*1000);

    if(type==8){type=0;} // Position control
    else if(type==0){type=1;} // Current control
    else if(type==9){type=2;} // FOC-Position control
    else if(type==10){type=3;} // FOC-Current control

    RBJoint_Request_Gain(type);
}
void    RBMotorControllerCOCOA::RBJoint_Set_PWM_RATE_LIMIT(double persent)
{
    uint16_t per = persent*100;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 3;
    memcpy(&(mb.data[1]),&per,2);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);

    usleep(50*1000);
    RBJoint_Request_PWM_RATE_LIMIT();
}
void    RBMotorControllerCOCOA::RBJoint_Set_FINDHOME_PARAMS(uint16_t search_vel, bool search_direction, uint16_t offset, bool off_direction)
{

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 14;
    memcpy(&(mb.data[1]),&search_vel,2);
    if(search_direction){mb.data[3] = 1;}
    else{mb.data[3] = 0;}
    memcpy(&(mb.data[4]),&offset,2);
    if(off_direction){mb.data[6] = 1;}
    else{mb.data[6] = 0;}
    mb.dlc = 7;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);

    usleep(50*1000);
    RBJoint_Request_FINDHOME_PARAMS();
}
void    RBMotorControllerCOCOA::RBJoint_Set_FINDHOME_AUTO_OFF()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 15;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(50*1000);
    RBJoint_Request_FINDHOME_PARAMS();
}
void    RBMotorControllerCOCOA::RBJoint_Set_Current_LIM(double lim)
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 18;
    mb.data[1] = (uchar)lim;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(50*1000);
    RBJoint_Request_Current_LIM();
}
void    RBMotorControllerCOCOA::RBJoint_Set_Motor_Direction(bool direc)
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 20;
    if(direc){mb.data[1] = 1;}
    else{mb.data[1] = 0;}
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(50*1000);
    RBJoint_Request_Motor_Direction();
}

//Newly added by HSW
int     RBMotorControllerCOCOA::RBJoint_EnableCurrentNulling(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 2; // Current nulling
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int     RBMotorControllerCOCOA::RBJoint_EnableFOCNulling(int ch, int enable){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 6; // FOC nulling
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

void    RBMotorControllerCOCOA::RBJoint_FINDHOMEALL(int type)
{
    if(type!=0&&type!=1)
    {
        std::cout<<"type ERROR! should be 0 or 1"<<std::endl;
        std::cout<<"0 : limit-switch mode, 1 : current-limit mode"<<std::endl;
        return;
    }
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 5;    // Every boards - Find home start
    mb.data[1] = type; // 0: limit-switch mode, 1: current-limit mode

    mb.dlc = 2;
    mb.id = 99;        // Global command

    canHandler->RBCAN_WriteData(mb);
//    return;
}

void    RBMotorControllerCOCOA::RBJoint_Request_POS_GOTO()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 11;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==3){
        uint16_t cf;
        memcpy(&(cf),&(mb.data[1]),2);
        Joints[0].Cocoa_Data.VEL_LIMIT = cf;
        memcpy(&(cf),&(mb.data[3]),2);
        Joints[0].Cocoa_Data.ACC_LIMIT = cf;
        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQPWMLIM_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_GainBACKEMF()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 22;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==7){
        Joints[0].Cocoa_Data.EMF_KP = mb.data[1]/100.0;
        cout <<"Joints[0].Cocoa_Data.EMF_KP : " << (double)mb.data[1]/100.0 << endl;
        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQMD_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_Gain_Override_Value()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 30;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==10){
        uint16_t cf;
        memcpy(&(cf),&(mb.data[1]),2);
        Joints[0].Cocoa_Data.GAIN_OVER_VALUE =cf;
        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQMD_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_Error_LIM()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 26;
    mb.data[1] = 0;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==8){

        uint16_t cf;
        memcpy(&(cf),&(mb.data[2]),2);
        Joints[0].Cocoa_Data.ERROR_LIM = cf;
        cout <<"ERRORLIM : " << double(cf) << endl;
        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQMD_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}
void    RBMotorControllerCOCOA::RBJoint_Request_FINDHOME_LIM()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 28;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==9){
        Joints[0].Cocoa_Data.FINDHOME_LIM = mb.data[1];
        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQMD_RMMC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}

void    RBMotorControllerCOCOA::RBJoint_Set_BACK_EMF_Gain(double Kp)
{
    uint16_t p = Kp*100;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 23;
    memcpy(&(mb.data[1]),&p,2);
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    RBJoint_Request_GainBACKEMF();

}
void    RBMotorControllerCOCOA::RBJoint_Set_POS_Rate_LIM(uint16_t vel_lim, uint16_t acc_lim)
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 12;
    memcpy(&(mb.data[1]),&vel_lim,2);
    memcpy(&(mb.data[3]),&acc_lim,2);
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);

    usleep(30*1000);
    RBJoint_Request_POS_GOTO();
}
void    RBMotorControllerCOCOA::RBJoint_Set_Error_LIM(char errtype,int limpulse)
{
    uint16_t lim = limpulse;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 27;
    mb.data[1] = errtype;
    memcpy(&(mb.data[2]),&lim,2);
    mb.dlc = 4;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(50*1000);
    RBJoint_Request_Error_LIM();
}
void    RBMotorControllerCOCOA::RBJoint_Set_FINDHOME_LIM(int numrot)
{
    if(numrot>100) numrot = 100;
    if(numrot<3) numrot = 3;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 29;
    mb.data[1] = numrot;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(50*1000);
    RBJoint_Request_FINDHOME_LIM();
}
void    RBMotorControllerCOCOA::RBJoint_Set_Gain_Override(int logscale,double mstime)
{
    uint16_t OVER_VALUE_0_TO_1000 = logscale;
    uint32_t MSEC_TIME10 = 10*mstime;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 31;
    memcpy(&(mb.data[1]),&OVER_VALUE_0_TO_1000,2);
    memcpy(&(mb.data[3]),&MSEC_TIME10,3);
    mb.dlc = 6;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);
    RBJoint_Request_Gain_Override_Value();
}

int     RBMotorControllerCOCOA::RBJoint_Init_ROM_Data(int ch){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 21;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int     RBMotorControllerCOCOA::RBJoint_Set_Error_Clear(int ch){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 32;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}
int     RBMotorControllerCOCOA::RBJoint_Torque_Nulling(int ch){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 50;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int     RBMotorControllerCOCOA::RBJoint_Change_Position_Mode(int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 47;
    mb.data[1] = mode; // 0: position, 1: velocity
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
int     RBMotorControllerCOCOA::RBJoint_Set_Position_Bound(char mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 35;
    mb.data[1] = mode; // 0: lower bound, 1: upper bound
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int     RBMotorControllerCOCOA::RBJoint_Set_Position_Limit_OnOff(char onoff){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 33;
    mb.data[1] = onoff; // 0: off, 1: on
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

void    RBMotorControllerCOCOA::RBJoint_Set_SixStep_Current_Nulling(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    mb.data[1] = 2;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFeedbackControlDirectly(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 1;
    }
    else
    {
        mb.data[1] = 0;
    }
//    mb.dlc = 2;
    mb.data[2] = 5;
    mb.dlc = 3;

    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}
int RBMotorControllerCOCOA::RBJoint_EnableFeedbackControl_CURRENTDirectly(int ch, int enable){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    if(enable)
    {
        mb.data[1] = 3;
    }
    else
    {
        mb.data[1] = 0;
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteDataDirectly(mb);
}


int RBMotorControllerCOCOA::RBBoard_ReadInformation(void){
    RBCAN_MB mb;

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    canHandler->RBCAN_SeeData(&mb);

    if(mb.status != RBCAN_NODATA)
    {
        if(mb.data[0]==0)
        {
            uint16_t cf;
            memcpy(&(cf),&(mb.data[2]),2);
            CANRate = cf;
            Joints[0].Cocoa_Data.FET_ONOFF = mb.data[4]&0b10000000;
            Joints[0].Cocoa_Data.BIGERROR_ONOFF = mb.data[4]&0b01000000;
            Joints[0].Cocoa_Data.ENCERROR_ONOFF = mb.data[4]&0b00100000;
            Joints[0].Cocoa_Data.CANERROR_ONOFF = mb.data[4]&0b00010000;
            Joints[0].Cocoa_Data.HOMEERROR_ONOFF = mb.data[4]&0b00001000;
            Joints[0].Cocoa_Data.BOARD_ACT = mb.data[5];
            Joints[0].Cocoa_Data.HOME_STATE = mb.data[6];
            mb.status = RBCAN_NODATA;
            canHandler->RBCAN_ReadData(&mb);
//            if(Joints[0].Cocoa_Data.BIGERROR_ONOFF)
//            {
//                FILE_LOG(logWARNING)<<BOARD_NAME.toStdString()<<" BIGERROR";
//            }
//            if(Joints[0].Cocoa_Data.ENCERROR_ONOFF)
//            {
//                FILE_LOG(logWARNING)<<BOARD_NAME.toStdString()<<" ENCERROR";
//            }
//            if(Joints[0].Cocoa_Data.CANERROR_ONOFF)
//            {
//                FILE_LOG(logWARNING)<<BOARD_NAME.toStdString()<<" CANERROR";
//            }
        }
        return true;
    }
    return false;
}




void    RBMotorControllerCOCOA::RBJoint_Set_BEMF(double EMF)
{
    int i=0;
    double tt = EMF;
    for(i=0;i<10;i++)
    {
        if(tt>99.9) {break;}
        tt = tt*10;
    }
    uint16_t p;
    if (i==9)
    {p = 0;}
    else
    {
        p = (int)(tt*10)+i;
    }

    FILE_LOG(logWARNING)<<EMF<<" "<<p<<endl;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 43;
    mb.data[1] = 2;
    memcpy(&(mb.data[2]),&p,2);
    mb.dlc = 4;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);

    RBJoint_Request_BEMF();

}

void    RBMotorControllerCOCOA::RBJoint_Request_BEMF()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 42;
    mb.data[1] = 2;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    canHandler->RBCAN_WriteData(mb);
    usleep(30*1000);
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    mb.status = RBCAN_NODATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA&&mb.data[0]==15){
        uint16_t p;
        memcpy(&p,&(mb.data[2]),2);
        double pp = p/10;
        int p2 = p%10;
        for(int i=0;i<p2;i++)
        {
            pp = pp/10;
        }
        Joints[0].Cocoa_Data.BEMF = pp;
        std::cout << ">>> REQBEMF:Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() <<" BEMF "<<pp<<" efef "<<p<< "\n";
        mb.status = RBCAN_NODATA;
    }
    else
    {
        std::cout << ">>> REQBEMF: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
        ConnectionStatus = false;
        return;
    }
}

/*////////////////////////////////////////////////////////////////////
            For Hydraulic Control Board (HCB)
/////////////////////////////////////////////////////////////////////*/
