#include "HCBSettingDialog.h"
#include "ui_HCBSettingDialog.h"

#include <QMessageBox>
#include <qvector2d.h>

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>

using namespace std;

HCBSettingDialog::HCBSettingDialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HCBSettingDialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("XXX");
}

HCBSettingDialog::~HCBSettingDialog()
{
    delete ui;
}

//void HCBSettingDialog::UpdateSettings(){
//    int mcId, mcCh, row;
//    QTableWidget *tw;
//    QString str;

//    QPalette palette_green;
//    palette_green.setColor(QPalette::Base,Qt::green);
//    QPalette palette_red;
//    palette_red.setColor(QPalette::Base,Qt::red);


//    for(int i=0; i<NO_OF_JOINTS; i++){
//        mcId = MC_GetID(i);
//        mcCh = MC_GetCH(i);
//        row = TW_ROW_Pairs[i].row;

//        switch(TW_ROW_Pairs[i].tw){
//        case 0:
//            tw = ui->TW_0;
//            break;
//        case 1:
//            tw = ui->TW_1;
//            break;
//        }

//        mSTAT stat = PODO_DATA.CoreSEN.MCStatus[mcId][mcCh];
//        str = "";
//        if(PODO_DATA.CoreSEN.ENCODER[mcId][mcCh].BoardConnection) str += "C  ";
//        else                                                      str += "N  ";
//        if(stat.b.HIP == 1) str += "H/"; // Open loop(PWM) Control mode
//        else                str += "-/";
//        if(stat.b.RUN == 1) str += "R/"; // Control ON
//        else                str += "-/";
//        str += QString().sprintf("%d", stat.b.HME); // Limit Sensor mode during Homing
//        tw->item(row, 0)->setText(str);
//        if(stat.b.RUN == 1 && stat.b.HME == 6){
//            tw->item(row, 0)->setBackgroundColor(QColor(100, 255, 100));    // green    Home(6) Run(on)
//        }else if(stat.b.HME != 0){
//            tw->item(row, 0)->setBackgroundColor(QColor(255, 100, 100));    // red      Home(x) Run(x)
//        }else{
//            tw->item(row, 0)->setBackgroundColor(QColor(255, 255, 100));    // yellow   Home(0)
//        }

//        str = "";
//        if(stat.b.JAM == 1) str += "JAM ";
//        if(stat.b.PWM == 1) str += "PWM ";
//        if(stat.b.BIG == 1) str += "BIG ";
//        if(stat.b.INP == 1) str += "INP ";
//        if(stat.b.FLT == 1) str += "FLT ";
//        if(stat.b.ENC == 1) str += "ENC ";
//        if(stat.b.CUR == 1) str += "CUR ";
//        if(stat.b.TMP == 1) str += "TMP ";
//        if(stat.b.PS1 == 1) str += "PS1 ";
//        if(stat.b.PS2 == 1) str += "PS2 ";
//        if(str == ""){
//            str = "-";
//            tw->item(row, 1)->setBackgroundColor(QColor(255, 255, 255));
//        }else{
//            tw->item(row, 1)->setBackgroundColor(QColor(255, 100, 100));
//        }
//        tw->item(row, 1)->setText(str);

//        tw->item(row, 2)->setText(QString().sprintf("%2d", (int)PODO_DATA.CoreSEN.MotorTemperature[mcId][mcCh]));
//        if(PODO_DATA.CoreSEN.MotorTemperature[mcId][mcCh] > 60)
//            tw->item(row, 2)->setBackgroundColor(QColor(255, 100, 100));
//        else
//            tw->item(row, 2)->setBackgroundColor(QColor(255, 255, 255));
//    }

//    ////COCOA_STATUS
//    int id = MC_GetID(lastSelected);
//    int ch = MC_GetCH(lastSelected);
//    id= lastSelected;
//    ch = 0;

//    int HV= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_SEARCH_VEL;
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_DIRECTION==false)
//    {HV = -HV;}
//    ui->LE_HOMEV2->setText(QString().sprintf("%d",HV));
//    int HO= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET;
//    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET_DIRECTION==false)
//    {HO = -HO;}
//    ui->LE_HOMEO2->setText(QString().sprintf("%d",HO));

//    ui->LE_BEMF_2->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.BEMF));

//    ui->LE_BEMF_3->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.EMF_KP));

//    ui->LE_HOMESEARCHLIM_2->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_LIM));

//    ui->LE_CURRENT_LIMIT_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.CURRENT_LIMIT));

//    double CL = PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.CURRENT_LIMIT;
//    ui->LE_CURRENT_LIMIT_2->setText(QString().sprintf("%.2f",CL));

//    ui->LE_MOTORDIRECTION->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.MOTOR_DIRECTION));

//    ui->LE_ERRORLIM_2->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.ERROR_LIM));



//    if(ui->RB_Pcon->isChecked())
//    {
//        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KP));
//        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KI));
//        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KD));
//    }
//    if(ui->RB_Ccon->isChecked())
//    {
//        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KP));
//        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KI));
//        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KD));
//    }
//    if(ui->RB_FOC_Pcon->isChecked())
//    {
//        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KP));
//        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KI));
//        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KD));
//    }
//    if(ui->RB_FOC_Ccon->isChecked())
//    {
//        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KP));
//        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KI));
//        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_C_KD));
//    }

//    int Mindex = 0;
//    int Mnrc2 = 0;
//    for(int i=0;i<12;i++)
//    {
//        if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(i)][0].NO_RESPONSE_CNT>Mnrc2)
//        {
//            Mindex = i;
//            Mnrc2 = PODO_DATA.CoreSEN.ENCODER[MC_GetID(i)][0].NO_RESPONSE_CNT;
//        }
//    }
//    ui->LE_NRC_2->setText(JointNameList[MC_GetID(Mindex)]+str.sprintf(":%d",Mnrc2));

//}

void HCBSettingDialog::on_BTN_CAN_CHECK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_CAN_CHECK;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_CAN_CHANNEL_ARRANGE_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_CAN_CHANNEL_ARRANGE;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_ENC_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to all joints
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = 2;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_VALVEPOS_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to all joints
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = 3;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_SOMETHING_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to all joints
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = 4;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_DISABLE_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to all joints
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = -1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

int HCBSettingDialog::GetSelected()
{
    if(ui->RB_RHR->isChecked()){return 0;}
    if(ui->RB_RHY->isChecked()){return 1;}
    if(ui->RB_RHP->isChecked()){return 2;}
    if(ui->RB_RKN->isChecked()){return 3;}
    if(ui->RB_RAP->isChecked()){return 4;}
    if(ui->RB_RAR->isChecked()){return 5;}
    if(ui->RB_LHR->isChecked()){return 6;}
    if(ui->RB_LHY->isChecked()){return 7;}
    if(ui->RB_LHP->isChecked()){return 8;}
    if(ui->RB_LKN->isChecked()){return 9;}
    if(ui->RB_LAP->isChecked()){return 10;}
    if(ui->RB_LAR->isChecked()){return 11;}
    if(ui->RB_WST->isChecked()){return 12;}
    return -1;
}

void HCBSettingDialog::on_BTN_ASK_EVERYTHING_clicked()
{
    int BNO = GetSelected();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = BNO; // Command to all joints
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_ASK_PARAMETERS;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_SHOW_EVERYTHING_clicked()
{
    int BNO = GetSelected();

    ui->LE_000->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.OPERATION_MODE));
    ui->LE_0001->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.SENSING_MODE));
    ui->LE_0002->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.CURRENT_CONTROL_MODE));

    ui->LE_001->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.CAN_FREQ));
    ui->LE_002->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.CONTROL_MODE));
    ui->LE_003->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINTENC_DIRECTION));
    ui->LE_004->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVEINPUT_DIRECTION));
    ui->LE_005->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVEENC_DIRECTION));
    ui->LE_006->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.BOARD_IN_VOLTAGE));
    ui->LE_007->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.BOARD_OPER_VOLTAGE));
    ui->LE_117->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF));

    ui->LE_008->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_DZ_PLUS));
    ui->LE_009->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_DZ_MINUS));
    ui->LE_010->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_CENTER_POS));
    ui->LE_011->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VOL_A));
    ui->LE_012->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VOL_B));
    ui->LE_013->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINTENC_PPP));
    ui->LE_014->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.FORCESEN_PPF));

    ui->LE_100->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.PIS_AREA_A));
    ui->LE_101->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.PIS_AREA_B));
    ui->LE_102->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VEL_COMPENSATION_K));
    ui->LE_103->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.ACTUATOR_COMPLIANCE_K));
    ui->LE_104->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_FEEDFORWARD));
    ui->LE_105->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.BULK_MODULUS));
    ui->LE_106->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.SUP_PRES));
    ui->LE_107->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.RET_PRES));
    ui->LE_108->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINTENC_LIMIT_PLUS));
    ui->LE_109->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINTENC_LIMIT_MINUS));
    ui->LE_110->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.PIS_STROKE));
    ui->LE_111->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVEPOS_LIMIT_PLUS));
    ui->LE_112->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVEPOS_LIMIT_MINUS));
    ui->LE_113_A->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.PRESSEN_PPP_A));
    ui->LE_113_B->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.PRESSEN_PPP_B));
    ui->LE_114->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.CONST_FRIC));
    ui->LE_115->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.HOMEPOS_OFFSET));
    ui->LE_116->setText(QString().sprintf("%d",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.HOMEPOS_VALVE_OPENING));

    ui->LE_200->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_P_KP));
    ui->LE_201->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_P_KI));
    ui->LE_202->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_P_KD));

    ui->LE_300->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_P_KP));
    ui->LE_301->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_P_KI));
    ui->LE_302->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_P_KD));

    ui->LE_400->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_F_KP));
    ui->LE_401->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_F_KI));
    ui->LE_402->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_F_KD));

    ui->LE_500->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_SPRING));
    ui->LE_501->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.JOINT_DAMPER));
}

void HCBSettingDialog::on_BTN_PARAMETER_SET_ALL_clicked()
{

    USER_COMMAND cmd;

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_SET_PARAMETERS;
    HCB_INFO HCB_Info;

    int BNO = GetSelected();
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = BNO;

    HCB_Info.OPERATION_MODE              = ui->LE_000->text().toInt();
    HCB_Info.SENSING_MODE                = ui->LE_0001->text().toInt();
    HCB_Info.CURRENT_CONTROL_MODE        = ui->LE_0002->text().toInt();

    HCB_Info.CAN_FREQ                    = ui->LE_001->text().toInt();
    HCB_Info.CONTROL_MODE                = ui->LE_002->text().toInt();
    HCB_Info.JOINTENC_DIRECTION          = ui->LE_003->text().toInt();
    HCB_Info.VALVEINPUT_DIRECTION        = ui->LE_004->text().toInt();
    HCB_Info.VALVEENC_DIRECTION          = ui->LE_005->text().toInt();
    HCB_Info.BOARD_IN_VOLTAGE            = ui->LE_006->text().toDouble();
    HCB_Info.BOARD_OPER_VOLTAGE          = ui->LE_007->text().toDouble();
    HCB_Info.VARIABLE_SUPPLYPRES_ONOFF   = ui->LE_117->text().toInt();

    HCB_Info.VALVE_DZ_PLUS               = ui->LE_008->text().toInt();
    HCB_Info.VALVE_DZ_MINUS              = ui->LE_009->text().toInt();
    HCB_Info.VALVE_CENTER_POS            = ui->LE_010->text().toInt();
    HCB_Info.VOL_A                       = ui->LE_011->text().toInt();
    HCB_Info.VOL_B                       = ui->LE_012->text().toInt();
    HCB_Info.JOINTENC_PPP                = ui->LE_013->text().toDouble();
    HCB_Info.FORCESEN_PPF                = ui->LE_014->text().toDouble();

    HCB_Info.PIS_AREA_A                  = ui->LE_100->text().toInt();
    HCB_Info.PIS_AREA_B                  = ui->LE_101->text().toInt();
    HCB_Info.VEL_COMPENSATION_K          = ui->LE_102->text().toInt();
    HCB_Info.ACTUATOR_COMPLIANCE_K       = ui->LE_103->text().toInt();
    HCB_Info.VALVE_FEEDFORWARD           = ui->LE_104->text().toInt();
    HCB_Info.BULK_MODULUS                = ui->LE_105->text().toInt();
    HCB_Info.SUP_PRES                    = ui->LE_106->text().toInt();
    HCB_Info.RET_PRES                    = ui->LE_107->text().toInt();
    HCB_Info.JOINTENC_LIMIT_PLUS         = ui->LE_108->text().toInt();
    HCB_Info.JOINTENC_LIMIT_MINUS        = ui->LE_109->text().toInt();
    HCB_Info.PIS_STROKE                  = ui->LE_110->text().toInt();
    HCB_Info.VALVEPOS_LIMIT_PLUS         = ui->LE_111->text().toInt();
    HCB_Info.VALVEPOS_LIMIT_MINUS        = ui->LE_112->text().toInt();
    HCB_Info.PRESSEN_PPP_A               = ui->LE_113_A->text().toDouble();
    HCB_Info.PRESSEN_PPP_B               = ui->LE_113_B->text().toDouble();
    HCB_Info.CONST_FRIC                  = ui->LE_114->text().toInt();
    HCB_Info.HOMEPOS_OFFSET              = ui->LE_115->text().toInt();
    HCB_Info.HOMEPOS_VALVE_OPENING       = ui->LE_116->text().toInt();

    HCB_Info.VALVE_P_KP                  = ui->LE_200->text().toDouble();
    HCB_Info.VALVE_P_KI                  = ui->LE_201->text().toDouble();
    HCB_Info.VALVE_P_KD                  = ui->LE_202->text().toDouble();

    HCB_Info.JOINT_P_KP                  = ui->LE_300->text().toDouble();
    HCB_Info.JOINT_P_KI                  = ui->LE_301->text().toDouble();
    HCB_Info.JOINT_P_KD                  = ui->LE_302->text().toDouble();

    HCB_Info.JOINT_F_KP                  = ui->LE_400->text().toDouble();
    HCB_Info.JOINT_F_KI                  = ui->LE_401->text().toDouble();
    HCB_Info.JOINT_F_KD                  = ui->LE_402->text().toDouble();

    HCB_Info.JOINT_SPRING                = ui->LE_500->text().toDouble();
    HCB_Info.JOINT_DAMPER                = ui->LE_501->text().toDouble();

    cmd.COMMAND_DATA.HCB_Info = HCB_Info;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);

}

void HCBSettingDialog::on_BTN_BOARD_TEST_ERRORRESET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_ERRORCLEAR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_SET_BNO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = GetSelected(); // Command to all joints
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_TARGET_BNO->text().toInt(); // Command to all joints
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_SET_BNO;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_ENC_ZERO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = GetSelected(); // Command to all joints
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_ENC_ZERO;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_FINDHOME_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to all joint
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_FINDHOME_FIRST_STAGE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -2; // Command to Waist, Ankle Joints (Light Findhome Process 1st stage)
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_FINDHOME_SECOND_STAGE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -3; // Command to Rest Joints (Light Findhome Process 2nd stage)
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_FINDHOME_ONEJOINT_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = GetSelected();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_TORQUEFORCE_NULLING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to selected joint
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_TORQUEFORCE_NULLING_ONE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = GetSelected(); // Command to selected joint
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_ALL_REFERENCE_RESET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REF_RESET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void HCBSettingDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SAVEDATA;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = true; // Start Saving
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_SAVE_END_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SAVEDATA;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false; // Finish Saving
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_READnSAVE_PARAMETERS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_READnSAVE_PARAMETERS;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HCBSettingDialog::on_BTN_LOADnSET_PARAMETERS_clicked()
{
    USER_COMMAND cmd;
    if(ui->RB_SETPARAMETERS_ONE->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_INT[0] = GetSelected(); // Command to selected joint
    } else if(ui->RB_SETPARAMETERS_ALL->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_INT[0] = -1; // Command to all joints
    }
    // Loaded Folder
    QString dir = QFileDialog::getExistingDirectory(this, "Select Directory", QDir::currentPath(), QFileDialog::ShowDirsOnly);
    dir = dir.right(41);
    qDebug() << dir;
    strcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, dir.toStdString().c_str());
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_LOADnSET_PARAMETERS;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

