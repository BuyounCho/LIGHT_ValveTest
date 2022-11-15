#ifndef RBMotorControllerCOCOA_H
#define RBMotorControllerCOCOA_H

#include "RBDataType.h"
#include "RBCAN.h"

//#include <math.h>
#include <cmath>

#define CONTROL_MODE_POS        0
#define CONTROL_MODE_CUR        2
#define CONTROL_MODE_PWM        3
#define CONTROL_MODE_CUR_POS    4 //GGmodified
#define CONTROL_MODE_FOC_CUR    5
#define CONTROL_MODE_FOC_POS    6
#define CONTROL_MODE_CUR_POS_FF    7 //GGmodified
#define CONTROL_MODE_DISABLED   (-1)


typedef struct _MOVE_JOINT_
{
    double			RefAngleCurrent;	// reference to move at this step
    double			RefAngleDelta;		// reference of the past step
    double			RefAngleToGo;		// goal position - initial position
    double			RefAngleInitial;	// initial position
    unsigned long	GoalTimeCount;		// the time at which the goal is reached
    unsigned long	CurrentTimeCount;	// current time count
    char			MoveFlag;			// move flag
} MOVE_JOINT, *pMOVE_JOINT;


class RBMotorControllerCOCOA
{
public:
    RBMotorControllerCOCOA();

    // from DB<-
    QString BOARD_NAME;
    int     BOARD_ID;
    int     CAN_CHANNEL;
    int     ID_SEND_GENERAL;
    int     ID_SEND_REF;
    int     ID_SEND_PWM;
    int     ID_SEND_REF_CURRENT;


    int     ID_RCV_GENERAL;
    int     ID_RCV_ENC;
    int     ID_RCV_CURRENT;
    int     ID_RCV_PWM;
    int     ID_RCV_TORQUE; // Seungwoo added


    // not from DB----
    int     ConnectionStatus;
    float   BoardTemperature;

    RB_JOINT    Joints[MAX_JOINT];
    MOVE_JOINT  MoveJoints[MAX_JOINT];


    void    RBMC_AddCANMailBox();
    void    RBBoard_GetDBData(DB_MC db);
    void    RBMC_SetFrictionParam();

    void    RBBoard_ReferenceOutEnable(bool _refEnable);
    int     RBBoard_CANCheck(int _canr);


    int     RBBoard_RequestEncoder(int mode);
    int     RBBoard_RequestCurrent(int mode);
    int     RBBoard_RequestPWM(int mode);

    int     RBJoint_ResetEncoder(int ch);
    int     RBJoint_EnableFeedbackControl_POSITION(int ch, int enable);
    int     RBJoint_EnableFeedbackControl_CURRENT(int ch, int enable);
    int     RBJoint_EnableFeedbackControl_CURRENTPOSITION(int ch, int enable);
    int     RBJoint_EnableFeedbackControl_FOC_POSITION(int ch, int enable);
    int     RBJoint_EnableFeedbackControl_FOC_CURRENT(int ch, int enable);
    int     RBJoint_EnableFOCPWMControl(int ch, int enable);
    int     RBJoint_EnablePWMControl(int ch, int enable);
    int     RBJoint_EnableFETDriver(int ch, int enable);
    int     RBJoint_EnableFeedbackControlDirectly(int ch, int enable);
    int     RBJoint_EnableFeedbackControl_CURRENTDirectly(int ch, int enable);


    int     RBBoard_SendReference(void);
    int     RBBoard_SendReferencePosition(int ref1);
    int     RBBoard_SendReferencePositionPlusFF(int ref1, double Current);
    int     RBBoard_SendReferenceCurrent(double ref1);
    int     RBBoard_SendReferenceFOCPosition(int ref1);
    int     RBBoard_SendReferenceFOCCurrent(double ref1, double ref2);
    int     RBBoard_SendReferencePWM(double ref1);
    int     RBBoard_SendReferenceCurrentPosition(int ref1);//GG added


    int     RBBoard_ReadEncoderData(void);
    int     RBBoard_ReadCurrentData(void);
    int     RBBoard_ReadPWMData(void);
    int     RBBoard_ReadTorqueSensorData(void);


    void    RBJoint_SetMoveJoint(int ch, float angle, float timeMs, int mode);
    void    RBJoint_MoveJoint(int ch);
    void    RBBoard_MoveJoint();
    int     RBBoard_Internal_Position_GOTO(int ref1);


    //COCOA_SPECIFIC?
    void    RBJoint_FINDHOME(int type);

    void    RBJoint_Request_Gain(int type);
    void    RBJoint_Request_Status();
    void    RBJoint_Request_PWM_RATE_LIMIT();
    void    RBJoint_Request_FINDHOME_PARAMS();
    void    RBJoint_Request_Current_LIM();
    void    RBJoint_Request_Motor_Direction();


    void    RBJoint_Set_Gain(int type, double Kp, double Ki, double Kd);
    void    RBJoint_Set_Gain_Auto_Tune(int type);
    void    RBJoint_Set_PWM_RATE_LIMIT(double persent);
    void    RBJoint_Set_FINDHOME_PARAMS(uint16_t search_vel, bool search_direction, uint16_t offset, bool off_direction);
    void    RBJoint_Set_FINDHOME_AUTO_OFF();
    void    RBJoint_Set_Current_LIM(double lim);
    void    RBJoint_Set_Motor_Direction(bool direc);

    //Newly added by HSW
    int     RBJoint_EnableCurrentNulling(int ch, int enable);
    int     RBJoint_EnableFOCNulling(int ch, int enable);
    void    RBJoint_FINDHOMEALL(int type);
    void    RBJoint_Request_POS_GOTO();
    void    RBJoint_Request_GainBACKEMF();
    void    RBJoint_Request_Gain_Override_Value();
    void    RBJoint_Request_Error_LIM();
    void    RBJoint_Request_FINDHOME_LIM();
    void    RBJoint_Set_BACK_EMF_Gain(double Kp);
    void    RBJoint_Set_POS_Rate_LIM(uint16_t vel_lim, uint16_t acc_lim);
    void    RBJoint_Set_Error_LIM(char errtype,int limpulse);
    void    RBJoint_Set_FINDHOME_LIM(int numrot);
    void    RBJoint_Set_Gain_Override(int logscale,double mstime);
    int     RBJoint_Init_ROM_Data(int ch);
    int     RBJoint_Set_Error_Clear(int ch);
    int     RBJoint_Torque_Nulling(int ch);
    int     RBJoint_Change_Position_Mode(int mode);
    int     RBJoint_Set_Position_Bound(char mode);
    int     RBJoint_Set_Position_Limit_OnOff(char onoff);
    void    RBJoint_Set_SixStep_Current_Nulling(void);

    void    RBJoint_Request_BEMF();
    void    RBJoint_Set_BEMF(double EMF);

    int     RBBoard_ReadInformation(void);

    //Added for Hydraulic Control Board(HCB)



    // used only once
    int     CANRate;
    int     Version;

private:
    bool    ReferenceOutEnable;

};

#endif // RBMotorControllerCOCOA_H
