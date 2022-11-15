#include "quadsettingdialog.h"
#include "ui_quadsettingdialog.h"

#include <QMessageBox>
#include <qvector2d.h>

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>
using namespace std;

//==============================//
// Joint Selection
//==============================//
enum _SET_JointSequentialNumber_0{
    S_RHR = 0, S_RHP, S_RKN, S_LHR, S_LHP, S_LKN,S_NUM_0
};

enum _SET_JointSequentialNumber_1{
    S_RSR = 0, S_RSP, S_REB, S_LSR, S_LSP, S_LEB, S_NUM_1
};

QString table_0_joint_name_[S_NUM_0] = {
    "RHY", "RHR", "RHP",
    "RKN", "RAP", "RAR"

};

QString table_1_joint_name_[S_NUM_1] = {
    "LHY", "LHR", "LHP",
    "LKN", "LAP", "LAR"
};

const struct {
    int tw;
    int row;
} TW_ROW_Pairs[NO_OF_JOINTS] = {
    {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5},
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5},
};

//==============================//
// Joint Motion
//==============================//
enum DM_CONTROL_COMMAND
{
    EMPTY_DM = 0,
    SIXSTEP_POSITION_CONTROL,
    SIXSTEP_CURRENT_CONTROL,
    SIXSTEP_CURRENT_POSITION_CONTROL,
    FOC_POSITION_CONTROL,
    FOC_VELOCITY_CONTROL,
    FOC_CURRENT_CONTROL,
    FOC_PWM_CONTROL,
    SINE_REFERENCE_CONTROL
};

/*
enum LM_CONTROL_COMMAND
{
    EMPTY_LM = 0,
    VELOCITY_CONTROL,
    CURRENT_CONTROL
};

enum EVAL_CONTROL_COMMAND
{
    EMPTY_EVAL = 0,
    VELOCITY_CURRENT,
    CURRENT_VELOCITY
};
*/

enum CONTROL_MODE_COMMAND
{
    POS_SET_MOVE = 100,
    POS_STOP_MOVE,
    POS_SET_REPETITIVE_MOVE,
    POS_STOP_REPETITIVE_MOVE,
    VEL_SET_JOGGING,
    VEL_STOP_JOGGING,
    CURRENT_SET_CMD,
    CURRENT_SET_CMD_ALL,
    CURRENT_STOP_CMD,
    PWM_SET_CMD,
    PWM_SET_CMD_ALL,
    PWM_STOP_CMD,
    VEL_SET_JOGGING_LM,
    VEL_STOP_JOGGING_LM,
    CURRENT_SET_CMD_LM,
    CURRENT_SET_CMD_ALL_LM,
    CURRENT_STOP_CMD_LM,
    FOC_NULLING,
    FOC_CONTROL_ENABLE,
    FOC_CONTROL_DISABLE,
    FOC_CONTROL_ENABLE_LM,
    FOC_CONTROL_DISABLE_LM,
    EVAL_RUN_VELOCITY_CURRENT,
    EVAL_RUN_CURRENT_VELOCITY,
    EVAL_STOP,
    UPDATE_PROTECTION_PARAMETER,
    LOGGING_DATA,
    LOGGING_STOP,
    SINE_REFERENCE_START,
    SINE_REFERENCE_STOP,
    PERFORMANCE_TEST_MOTION
};

enum REPETITIVE_MOTION_COMMAND
{
    REPETITIVE_OFF = 0,
    REPETITIVE_ON = 2
};

inline QVector2D CalcSpeedAcceleration(const double _period, const double _smooth, const double _angle, const int _mode){
    double _RefAngleInitial;
    double _RefAngleToGo;
    double _RefAngleDelta;

    _RefAngleInitial = PODO_DATA.CoreSEN.ENCODER[0][0].CurrentRefAngle;
    if(_mode==0){ //relative
        _RefAngleToGo = _RefAngleInitial + _angle;
    }else if(_mode==1){ //absolute
        _RefAngleToGo = _angle;
    }
    _RefAngleDelta = _RefAngleToGo-_RefAngleInitial;

    QVector2D temp;
    temp.setX(_RefAngleDelta/(_period-_smooth));//speed
    temp.setY(RBCORE_PI*temp.x()/(2.0f*_smooth)); //acceleration
    return temp;
}
inline double CalcAcceleration(const double _smooth, const double _speed){
    double _RefSpeedInitial;
    double _RefSpeedToGo;
    double _RefSpeedDelta;

    _RefSpeedInitial = PODO_DATA.CoreSEN.ENCODER[0][0].CurrentRefAngle;
    _RefSpeedToGo = _speed; // absolute
    _RefSpeedDelta = _RefSpeedToGo-_RefSpeedInitial;

    double temp;
    temp = 0.5f*_RefSpeedDelta*(RBCORE_PI/_smooth); //acceleration
    return temp;
}

//==============================//
QuadSettingDialog::QuadSettingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QuadSettingDialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("Quadruped");

    InitTable(ui->TW_0, table_0_joint_name_, S_NUM_0);\
    InitTable(ui->TW_1, table_1_joint_name_, S_NUM_1);

    lastSelected = RHR;
    select_working = false;
    ChangeSelectedJoint();

    for(int i=0;i<5;i++){
        DMtabAddress[i] = ui->TW_DriveMotorMotion->widget(i);
    }
    /*
    for(int i=0;i<2;i++){
        LMtabAddress[i] = ui->TW_LoadMotorMotion->widget(i);
    }
    */

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSettings()));
}

QuadSettingDialog::~QuadSettingDialog()
{
    delete ui;
}

void QuadSettingDialog::UpdateSettings(){
    int mcId, mcCh, row;
    QTableWidget *tw;
    QString str;

    QPalette palette_green;
    palette_green.setColor(QPalette::Base,Qt::green);
    QPalette palette_red;
    palette_red.setColor(QPalette::Base,Qt::red);

    int contact_fr = PODO_DATA.CoreSEN.CONTACT_SENSOR_POS[0];
    int contact_fl = PODO_DATA.CoreSEN.CONTACT_SENSOR_POS[1];
    int contact_br = PODO_DATA.CoreSEN.CONTACT_SENSOR_POS[2];
    int contact_bl = PODO_DATA.CoreSEN.CONTACT_SENSOR_POS[3];
    if(contact_fr <2400){
        ui->CONTACT_FR->setPalette(palette_green);
    }else{
        ui->CONTACT_FR->setPalette(palette_red);
    }
    if(contact_fl <2400){
        ui->CONTACT_FL->setPalette(palette_green);
    }else{
        ui->CONTACT_FL->setPalette(palette_red);
    }
    if(contact_br <2400){
        ui->CONTACT_BR->setPalette(palette_green);
    }else{
        ui->CONTACT_BR->setPalette(palette_red);
    }
    if(contact_bl <2400){
        ui->CONTACT_BL->setPalette(palette_green);
    }else{
        ui->CONTACT_BL->setPalette(palette_red);
    }
    ui->CONTACT_FR->setText(str.sprintf("%d", contact_fr));
    ui->CONTACT_FL->setText(str.sprintf("%d", contact_fl));
    ui->CONTACT_BR->setText(str.sprintf("%d", contact_br));
    ui->CONTACT_BL->setText(str.sprintf("%d", contact_bl));

    ui->SENSOR_IMU_ROLL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Roll));
    ui->SENSOR_IMU_PITCH->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Pitch));

    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = MC_GetID(i);
        mcCh = MC_GetCH(i);
        row = TW_ROW_Pairs[i].row;

        switch(TW_ROW_Pairs[i].tw){
        case 0:
            tw = ui->TW_0;
            break;
        case 1:
            tw = ui->TW_1;
            break;
        }

        mSTAT stat = PODO_DATA.CoreSEN.MCStatus[mcId][mcCh];
        str = "";
        if(PODO_DATA.CoreSEN.ENCODER[mcId][mcCh].BoardConnection) str += "C  ";
        else                                                      str += "N  ";
        if(stat.b.HIP == 1) str += "H/"; // Open loop(PWM) Control mode
        else                str += "-/";
        if(stat.b.RUN == 1) str += "R/"; // Control ON
        else                str += "-/";
        str += QString().sprintf("%d", stat.b.HME); // Limit Sensor mode during Homing
        tw->item(row, 0)->setText(str);
        if(stat.b.RUN == 1 && stat.b.HME == 6){
            tw->item(row, 0)->setBackgroundColor(QColor(100, 255, 100));    // green    Home(6) Run(on)
        }else if(stat.b.HME != 0){
            tw->item(row, 0)->setBackgroundColor(QColor(255, 100, 100));    // red      Home(x) Run(x)
        }else{
            tw->item(row, 0)->setBackgroundColor(QColor(255, 255, 100));    // yellow   Home(0)
        }

        str = "";
        if(stat.b.JAM == 1) str += "JAM ";
        if(stat.b.PWM == 1) str += "PWM ";
        if(stat.b.BIG == 1) str += "BIG ";
        if(stat.b.INP == 1) str += "INP ";
        if(stat.b.FLT == 1) str += "FLT ";
        if(stat.b.ENC == 1) str += "ENC ";
        if(stat.b.CUR == 1) str += "CUR ";
        if(stat.b.TMP == 1) str += "TMP ";
        if(stat.b.PS1 == 1) str += "PS1 ";
        if(stat.b.PS2 == 1) str += "PS2 ";
        if(str == ""){
            str = "-";
            tw->item(row, 1)->setBackgroundColor(QColor(255, 255, 255));
        }else{
            tw->item(row, 1)->setBackgroundColor(QColor(255, 100, 100));
        }
        tw->item(row, 1)->setText(str);

        //tw->item(row, 2)->setText(QString().sprintf("%2d", (int)PODO_DATA.CoreSEN.MCTemperature[mcId]));
        tw->item(row, 2)->setText(QString().sprintf("%2d", (int)PODO_DATA.CoreSEN.MotorTemperature[mcId][mcCh]));
        //if(PODO_DATA.CoreSEN.MCTemperature[mcId] > 60)
        if(PODO_DATA.CoreSEN.MotorTemperature[mcId][mcCh] > 60)
            tw->item(row, 2)->setBackgroundColor(QColor(255, 100, 100));
        else
            tw->item(row, 2)->setBackgroundColor(QColor(255, 255, 255));
    }

    ////COCOA_STATUS
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id= lastSelected;
    ch = 0;

    /*
    double PL = PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.PWM_RATE_LIMIT;
    ui->LE_PWM_RATE_LIMIT_2->setText(QString().sprintf("%.2f",PL));

    ui->LE_POSSETVEL_2->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.VEL_LIMIT));
    ui->LE_POSSETACC_2->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.ACC_LIMIT));
    */

    int HV= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_SEARCH_VEL;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_DIRECTION==false)
    {HV = -HV;}
    ui->LE_HOMEV2->setText(QString().sprintf("%d",HV));
    int HO= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET_DIRECTION==false)
    {HO = -HO;}
    ui->LE_HOMEO2->setText(QString().sprintf("%d",HO));

    ui->LE_BEMF_2->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.BEMF));

    ui->LE_BEMF_3->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.EMF_KP));

    ui->LE_HOMESEARCHLIM_2->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_LIM));

    ui->LE_CURRENT_LIMIT_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.CURRENT_LIMIT));

    double CL = PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.CURRENT_LIMIT;
    ui->LE_CURRENT_LIMIT_2->setText(QString().sprintf("%.2f",CL));

    ui->LE_MOTORDIRECTION->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.MOTOR_DIRECTION));

    /*
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.MOTOR_DIRECTION)
    {str = "Positive";}
    else
    {str = "Negative";}
    ui->LB_DIRECTION->setText(str + " direction");
    */
    /*
    ui->LE_BACK_EMF_GAIN2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.EMF_KP));
    */
    ui->LE_ERRORLIM_2->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.ERROR_LIM));
    /*
    ui->LE_GAINOVERRIDE_VAL2->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.GAIN_OVER_VALUE));

    ui->LE_FETONOFF  ->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.FET_ONOFF));
    ui->LE_ERROR_BIG ->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.BIGERROR_ONOFF));
    ui->LE_ERROR_CAN ->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.CANERROR_ONOFF));
    ui->LE_ERROR_ENC ->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.ENCERROR_ONOFF));
    ui->LE_ERROR_HOME->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.HOMEERROR_ONOFF));
    */
    /*
    ui->LB_FET_STATUS->setText(QString().sprintf("FET %d STATUS %d"
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.FET_ONOFF
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.BOARD_ACT));
    ui->LB_FET_STATUS->setText(QString().sprintf("FET %d STATUS %d HOME %d \nBE %d EE %d CE %d HE %d"
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.FET_ONOFF
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.BOARD_ACT
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.HOME_STATE
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.BIGERROR_ONOFF
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.ENCERROR_ONOFF
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.CANERROR_ONOFF
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.HOMEERROR_ONOFF));
    */
    /*
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.FET_ONOFF) ui->LE_FETONOFF->setStyleSheet("background-color: green");
    else ui->LE_FETONOFF->setStyleSheet("background-color: red");
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.BIGERROR_ONOFF) ui->LE_ERROR_BIG->setStyleSheet("background-color: red");
    else ui->LE_ERROR_BIG->setStyleSheet("background-color: green");
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.CANERROR_ONOFF) ui->LE_ERROR_CAN->setStyleSheet("background-color: red");
    else ui->LE_ERROR_CAN->setStyleSheet("background-color: green");
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.ENCERROR_ONOFF) ui->LE_ERROR_ENC->setStyleSheet("background-color: red");
    else ui->LE_ERROR_ENC->setStyleSheet("background-color: green");
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.HOMEERROR_ONOFF) ui->LE_ERROR_HOME->setStyleSheet("background-color: red");
    else ui->LE_ERROR_HOME->setStyleSheet("background-color: green");
    */

    if(ui->RB_Pcon->isChecked())
    {
        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KP));
        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KI));
        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KD));
    }
    if(ui->RB_Ccon->isChecked())
    {
        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KP));
        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KI));
        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KD));
    }
    if(ui->RB_FOC_Pcon->isChecked())
    {
        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KP));
        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KI));
        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KD));
    }
    if(ui->RB_FOC_Ccon->isChecked())
    {
        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KP));
        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KI));
        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KD));
    }

    //// Joint Motion
    /* ================================================================================== */
    /*
    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].BoardConnection==false){
        FLAG_FOC_Nulling = false;
        FLAG_DM_FOC_ENABLE = false;
    }
    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].BoardConnection==false){
        FLAG_LM_FOC_ENABLE = false;
    }
    */
    /* ================================================================================== */
    /*
    if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[0]){
        DisplayJointReference(RHR, ui->LE_STATUS_REF_DM);
    }else if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[1]){
        DisplayJointReference(RHR, ui->LE_STATUS_REF_DM);
    }else if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[2]){
        DisplayJointReferenceCurrent(RHR, ui->LE_STATUS_REF_DM);
    }else if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[3]){
        DisplayJointReferencePWM(RHR, ui->LE_STATUS_REF_DM);
    }

    if(ui->TW_LoadMotorMotion->currentWidget()==LMtabAddress[0]){
        DisplayJointReference(RKN, ui->LE_STATUS_REF_LM);
    }else if(ui->TW_LoadMotorMotion->currentWidget()==LMtabAddress[1]){
        DisplayJointReferenceCurrent(RKN, ui->LE_STATUS_REF_LM);
    }

    DisplayJointEncoderPosition(RHR, ui->LE_STATUS_POS_DM);
    DisplayJointEncoderPosition(RHP, ui->LE_STATUS_POS_LM);

    DisplayJointEncoderVelocity(RHR, ui->LE_STATUS_VEL_DM);
    DisplayJointEncoderVelocity(RHP, ui->LE_STATUS_VEL_LM);

    DisplayJointCurrent(RHR, ui->LE_STATUS_CUR_DM);

    DisplayJointTorque(RHP, ui->LCD_EVAL_DM_TORQUE, ui->LCD_EVAL_LM_TORQUE, ui->PB_EVAL_DM_TORQUE, ui->PB_EVAL_LM_TORQUE);
    DisplayJointEfficiency(RHP, ui->LCD_EVAL_EFFICIENCY, ui->PB_EVAL_EFFICIENCY);
    */
    /* ================================================================================== */
    /*
    if(DM_DRIVE_MODE!=EMPTY_DM && PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].BoardConnection){
        ui->LE_ENABLE_DM->setStyleSheet("background-color: green");
    }else{
        ui->LE_ENABLE_DM->setStyleSheet("background-color: red");
    }

    if(LM_DRIVE_MODE!=EMPTY_LM && PODO_DATA.CoreSEN.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].BoardConnection){
        ui->LE_ENABLE_LM->setStyleSheet("background-color: green");
    }else{
        ui->LE_ENABLE_LM->setStyleSheet("background-color: red");
    }

    if(EVAL_MODE!=EMPTY_EVAL && DM_DRIVE_MODE!=EMPTY_DM && LM_DRIVE_MODE!=EMPTY_LM){
        ui->LE_ENABLE_EVAL->setStyleSheet("background-color: green");
    }else{
        ui->LE_ENABLE_EVAL->setStyleSheet("background-color: red");
    }

    if(FLAG_FOC_Nulling) ui->LE_FOCNULLING_STATE_DM->setStyleSheet("background-color: green");
    else ui->LE_FOCNULLING_STATE_DM->setStyleSheet("background-color: red");
    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].Cocoa_Data.FET_ONOFF) ui->LE_FET_STATE_DM->setStyleSheet("background-color: green");
    else ui->LE_FET_STATE_DM->setStyleSheet("background-color: red");
    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].Cocoa_Data.BIGERROR_ONOFF) ui->LE_BIGERROR_STATE_DM->setStyleSheet("background-color: red");
    else ui->LE_BIGERROR_STATE_DM->setStyleSheet("background-color: green");
    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].Cocoa_Data.ENCERROR_ONOFF) ui->LE_ENCERROR_STATE_DM->setStyleSheet("background-color: red");
    else ui->LE_ENCERROR_STATE_DM->setStyleSheet("background-color: green");
    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].Cocoa_Data.CANERROR_ONOFF) ui->LE_CAN_STATE_DM->setStyleSheet("background-color: red");
    else ui->LE_CAN_STATE_DM->setStyleSheet("background-color: green");
    */
    /* ================================================================================== */

    int Mindex = 0;
    int Mnrc2 = 0;
    for(int i=0;i<12;i++)
    {
        if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(i)][0].NO_RESPONSE_CNT>Mnrc2)
        {
            Mindex = i;
            Mnrc2 = PODO_DATA.CoreSEN.ENCODER[MC_GetID(i)][0].NO_RESPONSE_CNT;
        }
    }
    ui->LE_NRC_2->setText(JointNameList[MC_GetID(Mindex)]+str.sprintf(":%d",Mnrc2));

}


//// Joint Selection
void QuadSettingDialog::InitTable(QTableWidget *table, QString j_names[], int num){
    QFont tableFont;
    tableFont.setPointSize(8);

    const int item_height = 30;
    const int item_width = 50;
    const int col_0_width = 60;
    const int col_1_width = 100;
    const int col_2_width = 30;


    // Horizontal - Column
    for(int i=0; i<3; i++){
        table->insertColumn(i);
        table->setHorizontalHeaderItem(i, new QTableWidgetItem());
        table->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->horizontalHeaderItem(i)->setFont(tableFont);
    }
    table->horizontalHeaderItem(0)->setSizeHint(QSize(col_0_width, item_height));
    table->horizontalHeaderItem(1)->setSizeHint(QSize(col_1_width, item_height));
    table->horizontalHeaderItem(2)->setSizeHint(QSize(col_2_width, item_height));
    table->setColumnWidth(0, col_0_width);
    table->setColumnWidth(1, col_1_width);
    table->setColumnWidth(2, col_2_width);
    table->horizontalHeaderItem(0)->setText("Status");
    table->horizontalHeaderItem(1)->setText("Error");
    table->horizontalHeaderItem(2)->setText("T");

    // Vertical - Row
    for(int i=0; i<num; i++){
        table->insertRow(i);
        table->setRowHeight(i,30);
        table->setVerticalHeaderItem(i, new QTableWidgetItem());
        table->verticalHeaderItem(i)->setText(j_names[i]);
        table->verticalHeaderItem(i)->setSizeHint(QSize(item_width, item_height));
        table->verticalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->verticalHeaderItem(i)->setFont(tableFont);
    }

    for(int i=0; i<num; i++){
        for(int j=0; j<3; j++){
            table->setItem(i, j, new QTableWidgetItem());
            table->item(i,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
            table->item(i,j)->setFlags(table->item(i,j)->flags() & ~Qt::ItemIsEditable);
            table->item(i,j)->setFont(tableFont);
        }
    }

    table->setMinimumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMaximumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMinimumHeight(30*(num+1) + 2);
    table->setMaximumHeight(30*(num+1) + 2);
    //table->resizeColumnsToContents();

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void QuadSettingDialog::UnselectOtherTable(int table){
    QTableWidget *tb;

    for(int i=0; i<7; i++){
        if(i == table)
            continue;

        switch(i){
        case 0:
            tb = ui->TW_0;
            break;
        case 1:
            tb = ui->TW_1;
            break;
        }

        QList<QTableWidgetItem*> itemlist = tb->selectedItems();
        for(int j=0; j<itemlist.size(); j++){
            itemlist[j]->setSelected(false);
        }
    }
}

void QuadSettingDialog::on_TW_0_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(0, ui->TW_0->currentRow());
    UnselectOtherTable(0);
    ChangeSelectedJoint();
    select_working = false;
}

void QuadSettingDialog::on_TW_1_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(1, ui->TW_1->currentRow());
    UnselectOtherTable(1);
    ChangeSelectedJoint();
    select_working = false;
}

int QuadSettingDialog::FindLastSelected(int tw, int row){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(TW_ROW_Pairs[i].tw == tw && TW_ROW_Pairs[i].row == row){
            FILE_LOG(logINFO) << i;
            return i;
        }
    }
    return -1;
}

void QuadSettingDialog::ChangeSelectedJoint(){
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id= lastSelected;
    ch = 0;
    QString str;
    str.sprintf(" (%d,%d)",id,ch);
    ui->LB_SELECTED->setText("Selected: " + JointNameList[lastSelected]+str);
    /*
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.MOTOR_DIRECTION)
    {str = "Positive";}
    else
    {str = "Negative";}
    ui->LB_DIRECTION->setText(str + " direction");
    */
    ui->RB_Pcon->setChecked(true);
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());

    int HV= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_SEARCH_VEL;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_DIRECTION==false)
    {HV = -HV;}
    ui->LE_HOMEV2->setText(QString().sprintf("%d",HV));
    ui->LE_HOMEV->setText(ui->LE_HOMEV->text());
    int HO= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET_DIRECTION==false)
    {HO = -HO;}
    ui->LE_HOMEO2->setText(QString().sprintf("%d",HO));
    ui->LE_HOMEO->setText(ui->LE_HOMEO->text());
    /*
    double PL = PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.PWM_RATE_LIMIT;
    ui->LE_PWM_RATE_LIMIT_2->setText(QString().sprintf("%.2f",PL));
    ui->LE_PWM_RATE_LIMIT->setText(ui->LE_PWM_RATE_LIMIT_2->text());
    */
    double CL = PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.CURRENT_LIMIT;
    ui->LE_CURRENT_LIMIT_2->setText(QString().sprintf("%.2f",CL));
    ui->LE_CURRENT_LIMIT->setText(ui->LE_CURRENT_LIMIT_2->text());

}


//// Joint Motion
void QuadSettingDialog::on_BTN_FOC_NULLING_clicked()
{
    int id, ch;
    if(ui->CB_SELECT_ALL_JOINTS->isChecked()){
        id = -1; //All
    }else{
        id = MC_GetID(lastSelected);
        ch = MC_GetCH(lastSelected);
    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1; // On
    cmd.COMMAND_DATA.USER_COMMAND = FOC_NULLING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_ENC_ZERO_DM_clicked()
{
    int id, ch;
    if(ui->CB_SELECT_ALL_JOINTS->isChecked()){
        id = -1; //All
    }else{
        id = MC_GetID(lastSelected);
        ch = MC_GetCH(lastSelected);
    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; //all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_ENCODER_ZERO;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
/*
void QuadSettingDialog::on_BTN_ENC_ZERO_LM_clicked()
{
    int id, ch;
    if(ui->CB_SELECT_ALL_JOINTS->isChecked()){
        id = -1; //All
    }else{
        id = MC_GetID(lastSelected);
        ch = MC_GetCH(lastSelected);
    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 1
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; //all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_RESET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
*/
void QuadSettingDialog::on_CBB_DriveMode_currentIndexChanged(int index)
{
//    QMessageBox::information(this,"Title",QString::number(index));
    switch(index){
    case EMPTY_DM:
    {
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[0],"Position");
        ui->TW_DriveMotorMotion->insertTab(1,DMtabAddress[1],"Velocity");
        ui->TW_DriveMotorMotion->insertTab(2,DMtabAddress[2],"Current");
        ui->TW_DriveMotorMotion->insertTab(3,DMtabAddress[3],"PWM");
        ui->TW_DriveMotorMotion->insertTab(4,DMtabAddress[4],"Sine Reference");
        ui->TW_DriveMotorMotion->setCurrentIndex(0);
        break;
    }
    case SIXSTEP_POSITION_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[0],"Position");
        break;
    }
    case SIXSTEP_CURRENT_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[2],"Current");
        break;
    }
    case SIXSTEP_CURRENT_POSITION_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[0],"Position");
        break;
    }
    case FOC_POSITION_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[0],"Position");
        break;
    }
    case FOC_VELOCITY_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[1],"Velocity");
        break;
    }
    case FOC_CURRENT_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[2],"Current");
        break;
    }
    case FOC_PWM_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[3],"PWM");
        break;
    }
    case SINE_REFERENCE_CONTROL:
    {
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->removeTab(0);
        ui->TW_DriveMotorMotion->insertTab(0,DMtabAddress[4],"Sine Reference");
        break;
    }
    default:
        break;
    }

    if(DM_DRIVE_MODE!=EMPTY_DM){
        QMessageBox::information(this,"Error","Disable the drive mode you are running before change to another mode");
        ui->CBB_DriveMode->setCurrentIndex(DM_DRIVE_MODE);
    }
    /*
    if(EVAL_MODE!=EMPTY_EVAL){
        QMessageBox::information(this,"WARNING","Evaluation is going on... Wait until it is done, otherwise press stop button manually to stop the evaluation.");
        ui->CBB_DriveMode->setCurrentIndex(DM_DRIVE_MODE);
    }
    */
}
/*
void QuadSettingDialog::on_TW_DriveMotorMotion_currentChanged(int index)
{
    if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[0]){
        ui->LB_REFERENCE_CMD_DM->setText("Position Ref. [deg]");
    }else if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[1]){
        ui->LB_REFERENCE_CMD_DM->setText("Velocity Ref. [deg/s]");
    }else if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[2]){
        ui->LB_REFERENCE_CMD_DM->setText("Current Ref. [Amp.]");
    }else if(ui->TW_DriveMotorMotion->currentWidget()==DMtabAddress[3]){
        ui->LB_REFERENCE_CMD_DM->setText("PWM Ref. [%]");
    }
}
*/
void QuadSettingDialog::on_BTN_ENABLE_DM_clicked()
{
    /*
    QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
    QMessageBox::information(this,"Title",QString::number(ui->CBB_DriveMode->currentIndex()));

    QVariant qv;
    QVariant qv2;
    QString qs;
    qv = ui->CBB_DriveMode->currentData();
    qv2 = ui->CBB_DriveMode->currentIndex();
    qs = ui->CBB_DriveMode->currentText();
    cout << "ComboBox currentData: " << qv.toInt() << endl;
    cout << "ComboBox currentIndex: " << qv2.toInt() << endl;
    cout << "ComboBox currentText: " << qs.toStdString() << endl;
    */

    int id, ch;
    if(ui->CB_SELECT_ALL_JOINTS->isChecked()){
        id = -1; //All
    }else{
        id = MC_GetID(lastSelected);
        ch = MC_GetCH(lastSelected);
    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1; // off=0, on=1

    if(PODO_DATA.CoreSEN.ENCODER[id][ch].BoardConnection==false){
        QMessageBox::information(this,"Error","Board Connection Failed");
        return;
    }
    if(FLAG_FOC_Nulling==false){
        /*
        QMessageBox::information(this,"Title","FOC_Nulling... Wait for 2 secs...");
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FOC_NULLING;
        cmd.COMMAND_TARGET = RBCORE_PODO_NO;
        pLAN->SendCommand(cmd);
        while(PODO_DATA.CoreCMD.COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT){
            usleep(2100*1000);
        }
        usleep(50*1000);
        */
        FLAG_FOC_Nulling = true;
    }
    switch(ui->CBB_DriveMode->currentIndex()){
    case EMPTY_DM:
    {
        QMessageBox::information(this,"Error","Please select a drive mode");
        /*
        QMessageBox::information(this,"Title","FOC_Nulling... Wait for 2 secs...");
        FLAG_FOC_Nulling = true;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_NULLING;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        */
        break;
    }
    case SIXSTEP_POSITION_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = SIXSTEP_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = SIXSTEP_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case SIXSTEP_CURRENT_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = SIXSTEP_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = SIXSTEP_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case SIXSTEP_CURRENT_POSITION_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = SIXSTEP_CURRENT_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = SIXSTEP_CURRENT_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_POSITION_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = FOC_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_VELOCITY_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = FOC_VELOCITY_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_VELOCITY_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_CURRENT_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = FOC_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_PWM_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = FOC_PWM_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_PWM_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case SINE_REFERENCE_CONTROL:
    {
//        QMessageBox::information(this,"Title",ui->CBB_DriveMode->currentText());
        FLAG_DM_FOC_ENABLE = true;
        DM_DRIVE_MODE = SINE_REFERENCE_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    default:
        break;
    }
}

void QuadSettingDialog::on_BTN_DISABLE_DM_clicked()
{
    int id, ch;
    if(ui->CB_SELECT_ALL_JOINTS->isChecked()){
        id = -1; //All
    }else{
        id = MC_GetID(lastSelected);
        ch = MC_GetCH(lastSelected);
    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0; // off=0, on=1

    FLAG_FOC_Nulling = false;

    switch(ui->CBB_DriveMode->currentIndex()){
    case SIXSTEP_POSITION_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = SIXSTEP_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case SIXSTEP_CURRENT_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = SIXSTEP_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case SIXSTEP_CURRENT_POSITION_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = SIXSTEP_CURRENT_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_POSITION_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_POSITION_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_VELOCITY_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_VELOCITY_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_CURRENT_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case FOC_PWM_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_PWM_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case SINE_REFERENCE_CONTROL:
    {
        FLAG_DM_FOC_ENABLE = false;
        DM_DRIVE_MODE = EMPTY_DM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = FOC_CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    default:
        break;
    }
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FET_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_POS_MOV_ABS_1_clicked()
{
    if(ui->CB_POS_ABS_REPETITIVE->isChecked()){
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double InitialAngle = ui->LE_POS_MOV_ABS_2->text().toDouble();
        double FinalAngle = ui->LE_POS_MOV_ABS_1->text().toDouble();
        double Mode = 1; // 0=relative, 1=absolute
        QVector2D temp;
        double Angle = fabs(FinalAngle - InitialAngle);
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,0);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = InitialAngle;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = FinalAngle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
//        if(ui->CBB_DriveMode->currentIndex()==SIXSTEP_POSITION_CONTROL){
//            cmd.COMMAND_DATA.USER_COMMAND = POS_SET_REPETITIVE_MOVE;
//        }
//        else if(ui->CBB_DriveMode->currentIndex()==FOC_POSITION_CONTROL){
//            cmd.COMMAND_DATA.USER_COMMAND = POS_SET_REPETITIVE_MOVE;
//        }
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_REPETITIVE_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }else{
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double Angle = ui->LE_POS_MOV_ABS_1->text().toDouble();
        double Mode = 1; // 0=relative, 1=absolute
        QVector2D temp;
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,Mode);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = Angle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }

}

void QuadSettingDialog::on_BTN_POS_MOV_ABS_2_clicked()
{
    if(ui->CB_POS_ABS_REPETITIVE->isChecked()){
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double InitialAngle = ui->LE_POS_MOV_ABS_1->text().toDouble();
        double FinalAngle = ui->LE_POS_MOV_ABS_2->text().toDouble();
        double Mode = 1; // 0=relative, 1=absolute
        QVector2D temp;
        double Angle = fabs(FinalAngle - InitialAngle);
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,0);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = InitialAngle;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = FinalAngle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_REPETITIVE_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }else{
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double Angle = ui->LE_POS_MOV_ABS_2->text().toDouble();
        double Mode = 1; // 0=relative, 1=absolute
        QVector2D temp;
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,Mode);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = Angle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_POS_MOV_REL_CCW_clicked()
{
    if(ui->CB_POS_REL_REPETITIVE->isChecked()){
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double Angle = ui->LE_POS_MOV_REL->text().toDouble();
        double Mode = 0; // 0=relative, 1=absolute
        QVector2D temp;
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,Mode);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = -Angle;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = Angle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_REPETITIVE_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }else{
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double Angle = ui->LE_POS_MOV_REL->text().toDouble();
        double Mode = 0; // 0=relative, 1=absolute
        QVector2D temp;
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,Mode);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = Angle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_POS_MOV_REL_CW_clicked()
{
    if(ui->CB_POS_REL_REPETITIVE->isChecked()){
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double Angle = ui->LE_POS_MOV_REL->text().toDouble();
        double Mode = 0; // 0=relative, 1=absolute
        QVector2D temp;
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,Mode);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = Angle;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = -Angle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_REPETITIVE_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }else{
        double Period = ui->DSB_POS_PERIOD->text().toDouble();
        double Smooth = ui->DSB_POS_SMOOTH->text().toDouble();
        double Angle = ui->LE_POS_MOV_REL->text().toDouble();
        double Mode = 0; // 0=relative, 1=absolute
        QVector2D temp;
        temp = CalcSpeedAcceleration(Period,Smooth,Angle,Mode);
        ui->LE_POS_SPEED->setText(QString::number(temp.x()));
        ui->LE_POS_ACCELERATION->setText(QString::number(temp.y()));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Period;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = -Angle;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Mode; // 0=relative, 1=absolute
        cmd.COMMAND_DATA.USER_COMMAND = POS_SET_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_POS_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = POS_STOP_MOVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_CB_POS_ABS_REPETITIVE_stateChanged(int arg1)
{
//    QMessageBox::information(this,"Title",QString::number(arg1));

    if(arg1==REPETITIVE_OFF){
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false;
        cmd.COMMAND_DATA.USER_COMMAND = POS_STOP_REPETITIVE_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_CB_POS_REL_REPETITIVE_stateChanged(int arg1)
{
//    QMessageBox::information(this,"Title",QString::number(arg1));
    if(arg1==REPETITIVE_OFF){
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false;
        cmd.COMMAND_DATA.USER_COMMAND = POS_STOP_REPETITIVE_MOVE;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_VEL_JOG_CCW_pressed()
{
    double Smooth = ui->DSB_VEL_SMOOTH->text().toDouble();
    double Speed = ui->DSB_VEL_SPEED->text().toDouble();
    double temp;
    temp = CalcAcceleration(Smooth,Speed);
    ui->LE_VEL_ACCELERATION->setText(QString::number(temp));

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
    cmd.COMMAND_DATA.USER_COMMAND = VEL_SET_JOGGING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_VEL_JOG_CCW_released()
{
    if(ui->CB_VEL_RUN_HELD->isChecked()){
        return;
    }else{
        double Smooth = ui->DSB_VEL_SMOOTH->text().toDouble();
        double Speed = 0.0;
        double temp;
        temp = CalcAcceleration(Smooth,Speed);
        ui->LE_VEL_ACCELERATION->setText(QString::number(temp));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
        cmd.COMMAND_DATA.USER_COMMAND = VEL_STOP_JOGGING;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_VEL_JOG_CW_pressed()
{
    double Smooth = ui->DSB_VEL_SMOOTH->text().toDouble();
    double Speed = ui->DSB_VEL_SPEED->text().toDouble();
    double temp;
    temp = CalcAcceleration(Smooth,Speed);
    ui->LE_VEL_ACCELERATION->setText(QString::number(temp));

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = -Speed;
    cmd.COMMAND_DATA.USER_COMMAND = VEL_SET_JOGGING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_VEL_JOG_CW_released()
{
    if(ui->CB_VEL_RUN_HELD->isChecked()){
        return;
    }else{
        double Smooth = ui->DSB_VEL_SMOOTH->text().toDouble();
        double Speed = 0.0;
        double temp;
        temp = CalcAcceleration(Smooth,Speed);
        ui->LE_VEL_ACCELERATION->setText(QString::number(temp));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
        cmd.COMMAND_DATA.USER_COMMAND = VEL_STOP_JOGGING;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_VEL_STOP_clicked()
{
    double Smooth = ui->DSB_VEL_SMOOTH->text().toDouble();
    double Speed = 0.0;
    double temp;
    temp = CalcAcceleration(Smooth,Speed);
    ui->LE_VEL_ACCELERATION->setText(QString::number(temp));

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
    cmd.COMMAND_DATA.USER_COMMAND = VEL_STOP_JOGGING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_CB_VEL_RUN_HELD_stateChanged(int arg1)
{
//    QMessageBox::information(this,"Title",QString::number(arg1));
    if(arg1==REPETITIVE_OFF){
        double Smooth = ui->DSB_VEL_SMOOTH->text().toDouble();
        double Speed = 0.0;
        double temp;
        temp = CalcAcceleration(Smooth,Speed);
        ui->LE_VEL_ACCELERATION->setText(QString::number(temp));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
        cmd.COMMAND_DATA.USER_COMMAND = VEL_STOP_JOGGING;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_1_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_1->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_2_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_3_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_3->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_4_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_4->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_5_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_5->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_6_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_6->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_7_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_7->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_8_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_8->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_9_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_9->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_10_clicked()
{
    int id = MC_GetID(lastSelected);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_10->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = id;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_CUR_CMD_1->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_CUR_CMD_3->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_CUR_CMD_4->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_CUR_CMD_5->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->LE_CUR_CMD_6->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->LE_CUR_CMD_7->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->LE_CUR_CMD_8->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->LE_CUR_CMD_9->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->LE_CUR_CMD_10->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->DSB_CUR_DURATION->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_ALL;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_STOP_clicked()
{
    double Current = 0.0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Current;
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_STOP_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_1->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_2->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_3->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_4->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_5_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_5->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_6_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_6->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_7_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_7->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_8_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_8->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_9_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_9->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_10_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_10->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_SET_CMD_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_PWM_CMD_1->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_PWM_CMD_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_PWM_CMD_3->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_PWM_CMD_4->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_PWM_CMD_5->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->LE_PWM_CMD_6->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->LE_PWM_CMD_7->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->LE_PWM_CMD_8->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->LE_PWM_CMD_9->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->LE_PWM_CMD_10->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->DSB_PWM_DURATION->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PWM_SET_CMD_ALL;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_PWM_STOP_clicked()
{
    double PWM = 0.0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_PWM_SMOOTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = PWM;
    cmd.COMMAND_DATA.USER_COMMAND = PWM_STOP_CMD;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

/*
void QuadSettingDialog::on_CBB_DriveMode_LM_currentIndexChanged(int index)
{
//    QMessageBox::information(this,"Title",QString::number(index));
    switch(index){
    case EMPTY_LM:
    {
        ui->TW_LoadMotorMotion->insertTab(0,LMtabAddress[0],"Velocity");
        ui->TW_LoadMotorMotion->insertTab(1,LMtabAddress[1],"Current");
        ui->TW_LoadMotorMotion->setCurrentIndex(0);
        break;
    }
    case VELOCITY_CONTROL:
    {
        ui->TW_LoadMotorMotion->removeTab(0);
        ui->TW_LoadMotorMotion->removeTab(0);
        ui->TW_LoadMotorMotion->insertTab(0,LMtabAddress[0],"Velocity");
        break;
    }
    case CURRENT_CONTROL:
    {
        ui->TW_LoadMotorMotion->removeTab(0);
        ui->TW_LoadMotorMotion->removeTab(0);
        ui->TW_LoadMotorMotion->insertTab(0,LMtabAddress[1],"Current");
        break;
    }
    default:
        break;
    }

    if(LM_DRIVE_MODE!=EMPTY_LM){
        QMessageBox::information(this,"WARNING","Disable the drive mode you are running before change to another mode");
        ui->CBB_DriveMode_LM->setCurrentIndex(LM_DRIVE_MODE);
    }
    if(EVAL_MODE!=EMPTY_EVAL){
        QMessageBox::information(this,"WARNING","Evaluation is going on... Wait until it is done, otherwise press stop button manually to stop the evaluation.");
        ui->CBB_DriveMode_LM->setCurrentIndex(LM_DRIVE_MODE);
    }
}

void QuadSettingDialog::on_TW_LoadMotorMotion_currentChanged(int index)
{
//    QMessageBox::information(this,"Title",QString::number(index));
//    if(index==0){
//        ui->LB_REFERENCE_CMD_LM->setText("Velocity Ref. [deg/s]");
//    }else if(index==1){
//        ui->LB_REFERENCE_CMD_LM->setText("Current Ref. [Amp]");
//    }
    if(ui->TW_LoadMotorMotion->currentWidget()==LMtabAddress[0]){
        ui->LB_REFERENCE_CMD_LM->setText("Velocity Ref. [deg/s]");
    }else if(ui->TW_LoadMotorMotion->currentWidget()==LMtabAddress[1]){
        ui->LB_REFERENCE_CMD_LM->setText("Current Ref. [Amp]");
    }
}

void QuadSettingDialog::on_BTN_ENABLE_LM_clicked()
{
    int id = MC_GetID(RKN);
    int ch = MC_GetCH(RKN);
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1; // off=0, on=1

    if(PODO_DATA.CoreSEN.ENCODER[id][ch].BoardConnection==false){
        QMessageBox::information(this,"Error","Board Connection Failed");
        return;
    }

    switch(ui->CBB_DriveMode_LM->currentIndex()){
    case EMPTY_LM:
    {
        QMessageBox::information(this,"Error","Please select a drive mode");
        break;
    }
    case VELOCITY_CONTROL:
    {
        FLAG_LM_FOC_ENABLE = true;
        LM_DRIVE_MODE = VELOCITY_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = VELOCITY_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE_LM;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case CURRENT_CONTROL:
    {
        FLAG_LM_FOC_ENABLE = true;
        LM_DRIVE_MODE = CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_ENABLE_LM;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    default:
        break;
    }
}

void QuadSettingDialog::on_BTN_DISABLE_LM_clicked()
{
    int id = MC_GetID(RKN);
    int ch = MC_GetCH(RKN);
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0; // off=0, on=1

    switch(ui->CBB_DriveMode_LM->currentIndex()){
    case VELOCITY_CONTROL:
    {
        FLAG_LM_FOC_ENABLE = false;
        LM_DRIVE_MODE = EMPTY_LM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = VELOCITY_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE_LM;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case CURRENT_CONTROL:
    {
        FLAG_LM_FOC_ENABLE = false;
        LM_DRIVE_MODE = EMPTY_LM;
        cmd.COMMAND_DATA.USER_PARA_CHAR[3] = CURRENT_CONTROL;
        cmd.COMMAND_DATA.USER_COMMAND = FOC_CONTROL_DISABLE_LM;
        cmd.COMMAND_TARGET = ALNum;

        pLAN->SendCommand(cmd);
        break;
    }
    default:
        break;
    }

}

void QuadSettingDialog::on_BTN_VEL_JOG_CCW_LM_pressed()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentQCurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please disable current control mode before running velocity control mode.");
//        return;
//    }
    double Smooth = ui->DSB_VEL_SMOOTH_LM->text().toDouble();
    double Speed = ui->DSB_VEL_SPEED_LM->text().toDouble();
    double temp;
    temp = CalcAcceleration(Smooth,Speed);
    ui->LE_VEL_ACCELERATION_LM->setText(QString::number(temp));

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_VELOCITY_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = VEL_SET_JOGGING_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_VEL_JOG_CCW_LM_released()
{
    if(ui->CB_VEL_RUN_HELD_LM->isChecked()){
        return;
    }else{
        double Smooth = ui->DSB_VEL_SMOOTH_LM->text().toDouble();
        double Speed = 0.0;
        double temp;
        temp = CalcAcceleration(Smooth,Speed);
        ui->LE_VEL_ACCELERATION_LM->setText(QString::number(temp));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_VELOCITY_PARAM->text().toDouble();
        cmd.COMMAND_DATA.USER_COMMAND = VEL_STOP_JOGGING_LM;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_VEL_JOG_CW_LM_pressed()
{
    QMessageBox::information(this,"Title","Not supported for CSMS-30B Motor");
}

void QuadSettingDialog::on_BTN_VEL_JOG_CW_LM_released()
{
//    QMessageBox::information(this,"Title","Not supported for CSMS-30B Motor");
}

void QuadSettingDialog::on_CB_VEL_RUN_HELD_LM_stateChanged(int arg1)
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please set velocity reference to zero before running current control mode.");
//        return;
//    }
    if(arg1==REPETITIVE_OFF){
        double Smooth = ui->DSB_VEL_SMOOTH_LM->text().toDouble();
        double Speed = 0.0;
        double temp;
        temp = CalcAcceleration(Smooth,Speed);
        ui->LE_VEL_ACCELERATION_LM->setText(QString::number(temp));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_VELOCITY_PARAM->text().toDouble();
        cmd.COMMAND_DATA.USER_COMMAND = VEL_STOP_JOGGING_LM;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_VEL_STOP_LM_clicked()
{
    double Smooth = ui->DSB_VEL_SMOOTH_LM->text().toDouble();
    double Speed = 0.0;
    double temp;
    temp = CalcAcceleration(Smooth,Speed);
    ui->LE_VEL_ACCELERATION_LM->setText(QString::number(temp));

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Smooth;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Speed;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_VELOCITY_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = VEL_STOP_JOGGING_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_1_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_1->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_2_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_3_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_3->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_4_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_4->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_5_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_5->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_6_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_6->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_7_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_7->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_8_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_8->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_9_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_9->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_10_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_10->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_SET_CMD_LM_ALL_clicked()
{
//    int id = MC_GetID(RKN);
//    int ch = MC_GetCH(RKN);
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].CurrentReference!=0){
//        QMessageBox::information(this,"Warning","Please diable velocity control mode before running current control mode.");
//        return;
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_CUR_CMD_LM_1->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_CUR_CMD_LM_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_CUR_CMD_LM_3->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_CUR_CMD_LM_4->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_CUR_CMD_LM_5->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->LE_CUR_CMD_LM_6->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->LE_CUR_CMD_LM_7->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->LE_CUR_CMD_LM_8->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->LE_CUR_CMD_LM_9->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->LE_CUR_CMD_LM_10->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->DSB_CUR_DURATION_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[12] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_SET_CMD_ALL_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_CUR_STOP_LM_clicked()
{
    double Current = 0.0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Current;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_CURRENT_PARAM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = CURRENT_STOP_CMD_LM;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}



void QuadSettingDialog::on_CBB_EVALTUATIOM_MODE_currentIndexChanged(int index)
{
    switch(index){
    case EMPTY_EVAL:
    {
        ui->CBB_DriveMode->setCurrentIndex(EMPTY_DM);
        ui->CBB_DriveMode_LM->setCurrentIndex(EMPTY_LM);
        break;
    }
    case VELOCITY_CURRENT:
    {
        ui->CBB_DriveMode->setCurrentIndex(FOC_VELOCITY_CONTROL);
        ui->CBB_DriveMode_LM->setCurrentIndex(CURRENT_CONTROL);
        break;
    }
    case CURRENT_VELOCITY:
    {
        ui->CBB_DriveMode->setCurrentIndex(FOC_CURRENT_CONTROL);
        ui->CBB_DriveMode_LM->setCurrentIndex(VELOCITY_CONTROL);
        break;
    }
    default:
        break;
    }

//    if(EVAL_MODE!=EMPTY_EVAL){
//        QMessageBox::information(this,"WARNING","Evaluation is going on... Wait until it is done, otherwise press stop button manually to stop the evaluation.");
//        ui->CBB_EVALTUATIOM_MODE->setCurrentIndex(EVAL_MODE);
//    }
}

void QuadSettingDialog::on_BTN_EVAL_RUN_clicked()
{
//    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].BoardConnection==false){
//        QMessageBox::information(this,"Error","Drive Motor Board Connection Failed");
//        return;
//    }
//    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].BoardConnection==false){
//        QMessageBox::information(this,"Error","Load Motor Board Connection Failed");
//        return;
//    }

//    if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].BoardConnection==false){
//        QMessageBox::information(this,"Error","Torque Sensor Board Connection Failed");
//        return;
//    }

    switch(ui->CBB_EVALTUATIOM_MODE->currentIndex()){
    case EMPTY_EVAL:
    {
        break;
    }
    case VELOCITY_CURRENT:
    {
        EVAL_MODE = VELOCITY_CURRENT;
        DM_DRIVE_MODE = FOC_VELOCITY_CONTROL;
        LM_DRIVE_MODE = CURRENT_CONTROL;
        double VelSmooth = ui->DSB_VEL_SMOOTH->text().toDouble();
        double VelSpeed = ui->DSB_VEL_SPEED->text().toDouble();
        double temp;
        temp = CalcAcceleration(VelSmooth,VelSpeed);
        ui->LE_VEL_ACCELERATION->setText(QString::number(temp));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = VelSmooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = VelSpeed;

        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_CUR_CMD_LM_1->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_CUR_CMD_LM_2->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_CUR_CMD_LM_3->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->LE_CUR_CMD_LM_4->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->LE_CUR_CMD_LM_5->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->LE_CUR_CMD_LM_6->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->LE_CUR_CMD_LM_7->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->LE_CUR_CMD_LM_8->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->LE_CUR_CMD_LM_9->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->LE_CUR_CMD_LM_10->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[12] = ui->DSB_CUR_DURATION_LM->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[13] = ui->DSB_CUR_SMOOTH_LM->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[14] = ui->LE_LM_CURRENT_PARAM->text().toDouble();

        cmd.COMMAND_DATA.USER_COMMAND = EVAL_RUN_VELOCITY_CURRENT;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    case CURRENT_VELOCITY:
    {
        EVAL_MODE = VELOCITY_CURRENT;
        DM_DRIVE_MODE = FOC_CURRENT_CONTROL;
        LM_DRIVE_MODE = VELOCITY_CONTROL;
        double VelSmooth = ui->DSB_VEL_SMOOTH_LM->text().toDouble();
        double VelSpeed = ui->DSB_VEL_SPEED_LM->text().toDouble();
        double temp;
        temp = CalcAcceleration(VelSmooth,VelSpeed);
        ui->LE_VEL_ACCELERATION_LM->setText(QString::number(temp));

        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = VelSmooth;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = VelSpeed;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LM_VELOCITY_PARAM->text().toDouble();

        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_CUR_CMD_1->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_CUR_CMD_2->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->LE_CUR_CMD_3->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->LE_CUR_CMD_4->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->LE_CUR_CMD_5->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->LE_CUR_CMD_6->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->LE_CUR_CMD_7->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->LE_CUR_CMD_8->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->LE_CUR_CMD_9->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[12] = ui->LE_CUR_CMD_10->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[13] = ui->DSB_CUR_DURATION->text().toDouble();
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[14] = ui->DSB_CUR_SMOOTH->text().toDouble();

        cmd.COMMAND_DATA.USER_COMMAND = EVAL_RUN_CURRENT_VELOCITY;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
        break;
    }
    default:
        break;
    }

    cout << " ui->DSB_VEL_SMOOTH = " << ui->DSB_VEL_SMOOTH->text().toDouble() << endl;
    cout << "ui->DSB_VEL_SPEED = " << ui->DSB_VEL_SPEED->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_1 = " << ui->LE_CUR_CMD_LM_1->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_2 = " << ui->LE_CUR_CMD_LM_2->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_3 = " << ui->LE_CUR_CMD_LM_3->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_4 = " << ui->LE_CUR_CMD_LM_4->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_5 = " << ui->LE_CUR_CMD_LM_5->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_6 = " << ui->LE_CUR_CMD_LM_6->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_7 = " << ui->LE_CUR_CMD_LM_7->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_8 = " << ui->LE_CUR_CMD_LM_8->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_9 = " << ui->LE_CUR_CMD_LM_9->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_10 = " << ui->LE_CUR_CMD_LM_10->text().toDouble() << endl;
    cout << "ui->DSB_CUR_DURATION_LM = " << ui->DSB_CUR_DURATION_LM->text().toDouble() << endl;
    cout << "ui->DSB_CUR_SMOOTH_LM = " << ui->DSB_CUR_SMOOTH_LM->text().toDouble() << endl;
    cout << "ui->LE_LM_CURRENT_PARAM = " << ui->LE_LM_CURRENT_PARAM->text().toDouble() << endl;
}

void QuadSettingDialog::on_BTN_EVAL_STOP_clicked()
{
    int mode = EVAL_MODE;
    EVAL_MODE = EMPTY_EVAL;
    DM_DRIVE_MODE = EMPTY_DM;
    LM_DRIVE_MODE = EMPTY_LM;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = mode;
    cmd.COMMAND_DATA.USER_COMMAND = EVAL_STOP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);

    cout << " ui->DSB_VEL_SMOOTH = " << ui->DSB_VEL_SMOOTH->text().toDouble() << endl;
    cout << "ui->DSB_VEL_SPEED = " << ui->DSB_VEL_SPEED->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_1 = " << ui->LE_CUR_CMD_LM_1->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_2 = " << ui->LE_CUR_CMD_LM_2->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_3 = " << ui->LE_CUR_CMD_LM_3->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_4 = " << ui->LE_CUR_CMD_LM_4->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_5 = " << ui->LE_CUR_CMD_LM_5->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_6 = " << ui->LE_CUR_CMD_LM_6->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_7 = " << ui->LE_CUR_CMD_LM_7->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_8 = " << ui->LE_CUR_CMD_LM_8->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_9 = " << ui->LE_CUR_CMD_LM_9->text().toDouble() << endl;
    cout << "ui->LE_CUR_CMD_LM_10 = " << ui->LE_CUR_CMD_LM_10->text().toDouble() << endl;
    cout << "ui->DSB_CUR_DURATION_LM = " << ui->DSB_CUR_DURATION_LM->text().toDouble() << endl;
    cout << "ui->DSB_CUR_SMOOTH_LM = " << ui->DSB_CUR_SMOOTH_LM->text().toDouble() << endl;
    cout << "ui->LE_LM_CURRENT_PARAM = " << ui->LE_LM_CURRENT_PARAM->text().toDouble() << endl;
}



void QuadSettingDialog::on_BTN_UPDATE_PROTECTION_DATA_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_LIMIT_VEL_DM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_LIMIT_VEL_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_LIMIT_ACC_DM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_LIMIT_ACC_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_LIMIT_CUR_DM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->LE_LIMIT_CUR_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->LE_LIMIT_TORQUE_DM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->LE_LIMIT_TORQUE_LM->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = UPDATE_PROTECTION_PARAMETER;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_FOC_NULLING_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1; // On
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FOC_NULLING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_TORQUE_NULLING_clicked()
{
    int id = MC_GetID(RHP);
    int ch = MC_GetCH(RHP);
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_COCOA_TORQUE_NULLING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_REQUEST_STATE_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_COCOA_REQUEST_EVERYTHING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
*/

//// Parameter Setting

void QuadSettingDialog::on_RB_Pcon_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());
}

void QuadSettingDialog::on_RB_Ccon_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());
}

void QuadSettingDialog::on_RB_FOC_Pcon_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());
}

void QuadSettingDialog::on_RB_FOC_Ccon_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());
}

void QuadSettingDialog::on_BTN_LOGGING_DATA_clicked()
{
//    int id, ch;
//    if(ui->CB_SELECT_ALL_JOINTS->isChecked()){
//        id = -1; //All
//    }else{
//        id = MC_GetID(lastSelected);
//        ch = MC_GetCH(lastSelected);
//    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LOGGING_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_LOGGING_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LOGGING_STOP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}


void QuadSettingDialog::on_BTN_SINE_REFERENCE_START_clicked()
{
    USER_COMMAND cmd;
    if(ui->CB_SINE_REFERENCE_RUN_HELD->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = true; // off=0, on=1
    }else{
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false; // off=0, on=1
    }
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->DSB_SINE_REFERENCE_AMP_HIGH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->DSB_SINE_REFERENCE_FREQ_HIGH->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = SINE_REFERENCE_START;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_SINE_REFERENCE_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = SINE_REFERENCE_STOP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_CB_SINE_REFERENCE_RUN_HELD_stateChanged(int arg1)
{
    if(arg1==REPETITIVE_OFF){
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = SINE_REFERENCE_STOP;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void QuadSettingDialog::on_BTN_CAN_CHECK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_INIT_CHECK_DEVICE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void QuadSettingDialog::on_BTN_SENSOR_CONTACT_EN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_FT_NULL;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;//enable
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_SENSOR_CONTACT_DIS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_FT_NULL;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;//disable
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_SENSOR_IMU_EN_clicked()
{
    // imu enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_SENSOR_IMU_NULL_clicked()
{
    // imu nulling
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_SENSOR_IMU_ZERO_clicked()
{
    // imu set as zero
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 4;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void QuadSettingDialog::on_BTN_SENSOR_IMU_ZERO_2_clicked()
{
    // imu set as acc
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

