#include "SH_TASK_Dialog.h"
#include "ui_SH_TASK_Dialog.h"

#include <QMessageBox>
#include <qvector2d.h>

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>
using namespace std;

SH_TASK_Dialog::SH_TASK_Dialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SH_TASK_Dialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("SH_TASK");

//    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSettings()));
//    connect(displayTimer, SIGNAL(timeout()), this, SLOT(UpdateSettings()));
}

SH_TASK_Dialog::~SH_TASK_Dialog()
{
    delete ui;
}

void SH_TASK_Dialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = true;
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_SAVE_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = false; // save flag off
    string filename = ui->CON_SAVE_FILENAME->text().toStdString(); // filename string
    if(filename.empty()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = NULL;
    } else {
        strcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, filename.c_str());
    }
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

int SH_TASK_Dialog::Get_BNO_Selected()
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

///////////////////////////////////////////////////////////////////////////////////////////
/// Valve Identification
/// //////////////////////////////////////////////////////////////////////////////////////////

void SH_TASK_Dialog::on_BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->EDIT_LIGHT_VALVEID_OPEN_MIN->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->EDIT_LIGHT_VALVEID_OPEN_RESOL->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[2] = ui->EDIT_LIGHT_VALVEID_OPEN_MAX->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_LIGHT_VALVEID_PRES_MIN->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_LIGHT_VALVEID_PRES_RESOL->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->EDIT_LIGHT_VALVEID_PRES_MAX->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_VALVE_ID_POS;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->EDIT_LIGHT_VALVEID_OPEN_MIN->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->EDIT_LIGHT_VALVEID_OPEN_RESOL->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[2] = ui->EDIT_LIGHT_VALVEID_OPEN_MAX->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_LIGHT_VALVEID_PRES_MIN->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_LIGHT_VALVEID_PRES_RESOL->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->EDIT_LIGHT_VALVEID_PRES_MAX->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_VALVE_ID_NEG;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

//void SH_TASK_Dialog::on_BTN_LIGHT_ANKLE_VALVE_ID_MOOG_PARAMETER_clicked()
//{
//    QString str;
//    ui->LE_LIGHT_ANKLE_VALVE_ID_DATANUM->setText(str.sprintf("%d",100));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_DATASTEP->setText(str.sprintf("%d",10));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_FORWARD_OPEN->setText(str.sprintf("%d",1200));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_RETURN_OPEN->setText(str.sprintf("%d",3000));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_MID_OFFSET->setText(str.sprintf("%d",0));
//}

//void SH_TASK_Dialog::on_BTN_LIGHT_ANKLE_VALVE_ID_KNR_PARAMETER_clicked()
//{
//    QString str;
//    ui->LE_LIGHT_ANKLE_VALVE_ID_DATANUM->setText(str.sprintf("%d",100));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_DATASTEP->setText(str.sprintf("%d",30));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_FORWARD_OPEN->setText(str.sprintf("%d",500));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_RETURN_OPEN->setText(str.sprintf("%d",5000));
//    ui->LE_LIGHT_ANKLE_VALVE_ID_MID_OFFSET->setText(str.sprintf("%d",0));
//}

///////////////////////////////////////////////////////////////////////////////////////////
/// Spool Open Control (Openloop)
/// //////////////////////////////////////////////////////////////////////////////////////////

void SH_TASK_Dialog::on_BTN_CONST_OPEN_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_INT[2] = ui->LE_CONST_OPEN_VALUE->text().toInt(); // Opening Value
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_CONST_OPENING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_CONST_OPEN_NEG_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_INT[2] = -ui->LE_CONST_OPEN_VALUE->text().toInt(); // Opening Value
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_CONST_OPENING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_OPENLOOP_SINE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_OPENLOOP_SINE_TIME->text().toDouble(); // type : PWM
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_OPENLOOP_SINE_MAG->text().toDouble(); // type : PWM
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_OPENLOOP_SINE_NUM->text().toDouble(); // on
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_SINEWAVE_OPENING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}


///////////////////////////////////////////////////////////////////////////////////////////
/// Joint Control (Position, Torque)
/// //////////////////////////////////////////////////////////////////////////////////////////

void SH_TASK_Dialog::on_BTN_HOMEPOSE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_POSITION_CONTROL_HOMEPOSE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_WALKREADY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_POSITION_CONTROL_WALKREADYPOSE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_LIGHT_GOTO_POSITION_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Joint
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_GOTO_POSITION_POSITION->text().toDouble(); // Desired Position
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_GOTO_POSITION_TIME->text().toDouble(); // Moving Time

    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_POSITION_CONTROL_GOTO_POSE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_GOTO_RANDOM_POSITION_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Joint
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_GOTO_RANDOM_POSITION_TIME->text().toDouble(); // Moving Time

    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_POSITION_CONTROL_RANDOM_MOTION;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_POSITION_CONTROL_SINEWAVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Joint
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_POSITION_CONTROL_SINEWAVE_PERIOD->text().toDouble(); // type : PWM
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_POSITION_CONTROL_SINEWAVE_MAG->text().toDouble(); // type : PWM
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_POSITION_CONTROL_SINEWAVE_NUM->text().toDouble(); // on
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_POSITION_CONTROL_SINEWAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_TORQUE_CONTROL_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_REF_TORQUE->text().toDouble(); // Ref Torque
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_REF_TORQUE_KP_COMP->text().toDouble(); // Ref Torque
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_REF_TORQUE_KI_COMP->text().toDouble(); // Ref Torque
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_TORQUE_CONTROL_GO;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_TORQUE_CONTROL_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = Get_BNO_Selected(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_TORQUE_CONTROL_STOP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}


void SH_TASK_Dialog::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false; // RHY
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = false; // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->CBX_ROBOT_CONTROLMETHOD_RHP->isChecked(); // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = ui->CBX_ROBOT_CONTROLMETHOD_RKN->isChecked(); // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[4] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked(); // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[5] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked(); // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[6] = false; // LHY
    cmd.COMMAND_DATA.USER_PARA_CHAR[7] = false; // LHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[8] = ui->CBX_ROBOT_CONTROLMETHOD_LHP->isChecked(); // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[9] = ui->CBX_ROBOT_CONTROLMETHOD_LKN->isChecked(); // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[10] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked(); // RHR
    cmd.COMMAND_DATA.USER_PARA_CHAR[11] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked(); // RHR

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_POSorTOR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SH_TASK_Dialog::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS_clicked()
{
    ui->CBX_ROBOT_CONTROLMETHOD_RHP->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RKN->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RANK->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LHP->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LKN->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LANK->setChecked(0);
}

void SH_TASK_Dialog::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR_clicked()
{
    ui->CBX_ROBOT_CONTROLMETHOD_RHP->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RKN->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RANK->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LHP->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LKN->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LANK->setChecked(1);
}


void SH_TASK_Dialog::on_LE_VARIABLE_SUPPLY_SWING_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_JUMP_OPENING_RHP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->LE_JUMP_OPENING_RKN->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[2] = ui->LE_JUMP_OPENING_RAP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[3] = ui->LE_JUMP_OPENING_LHP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[4] = ui->LE_JUMP_OPENING_LKN->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[5] = ui->LE_JUMP_OPENING_LAP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_JUMP_OPENTIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_JUMP_RETURNTIME->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = SH_TASK_JUMP_OPENING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

