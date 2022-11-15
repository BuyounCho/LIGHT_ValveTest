#include "PumpControlDialog.h"
#include "ui_PumpControlDialog.h"

#include <QMessageBox>
#include <qvector2d.h>

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>
using namespace std;

PumpControlDialog::PumpControlDialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PumpControlDialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("PumpControl");
    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateDatas()));
}

PumpControlDialog::~PumpControlDialog()
{
    delete ui;
}

inline void DisplayPumpPressure(QLineEdit *edit){
    edit->setText(QString().sprintf("%.1f", PODO_DATA.CoreSEN.PUMP[0].CurrentPressure));
}

inline void DisplayPumpPressureReference(QLineEdit *edit){
    edit->setText(QString().sprintf("%.1f", PODO_DATA.CoreREF.PumpPressureReference[0]));
}

inline void DisplayPumpVelocity(QLineEdit *edit){
    edit->setText(QString().sprintf("%.1f", PODO_DATA.CoreSEN.PUMP[0].CurrentVelocity));
}

inline void DisplayPumpDutyReference(QLineEdit *edit){
    edit->setText(QString().sprintf("%.0f", PODO_DATA.CoreSEN.PUMP[0].CurrentRefVelocity));
}


inline void DisplayPumpTemperature(QLineEdit *edit){
    float temp = PODO_DATA.CoreSEN.PUMP[0].CurrentTemperature;
    edit->setText(QString().sprintf("%.1f", PODO_DATA.CoreSEN.PUMP[0].CurrentTemperature));

    QPalette *palette = new QPalette();
    if(temp < 40.0) {
        palette->setColor(QPalette::Text,Qt::blue);
    } else if (temp < 65.0) {
        palette->setColor(QPalette::Text,Qt::green);
    } else if (temp < 90.0) {
        palette->setColor(QPalette::Text,Qt::yellow);
    } else {
        palette->setColor(QPalette::Text,Qt::red);
    }
    edit->setPalette(*palette);
}



void PumpControlDialog::UpdateDatas(){
    DisplayPumpPressure(ui->EDIT_PUMP_PRESSURE);
    DisplayPumpPressureReference(ui->EDIT_PUMP_PRESSURE_REF);
    DisplayPumpVelocity(ui->EDIT_PUMP_VELOCITY);
    DisplayPumpDutyReference(ui->EDIT_PUMP_DUTYREFERENCE);
    DisplayPumpTemperature(ui->EDIT_PUMP_TEMPERATURE);
}

void PumpControlDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = true; // save flag on!
    cmd.COMMAND_DATA.USER_COMMAND = PUMP_CONTROL_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SAVE_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false; // save flag off!
    string filename = ui->CON_SAVE_FILENAME->text().toStdString(); // filename string
    if(filename.empty()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = NULL;
    } else {
        strcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, filename.c_str());
    }
    cmd.COMMAND_DATA.USER_COMMAND = PUMP_CONTROL_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SEND_DUTY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->CON_DUTY->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SEND_DUTY_ZERO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SEND_PRESSURE_NULL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = PumpController_GeneralMSG_CMD_PRESSURENULL;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_SEND_COMMAND;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SEND_PUMPDATA_REQUEST_ON_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = PumpController_GeneralMSG_CMD_DATAREQUESTFLAG;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1; // Request ON!
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_SEND_COMMAND;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SEND_PUMPDATA_REQUEST_OFF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = PumpController_GeneralMSG_CMD_DATAREQUESTFLAG;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0; // Request OFF!
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_SEND_COMMAND;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SEND_PUMP_CONTROL_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = PumpController_GeneralMSG_CMD_CTRL_ON;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_SEND_COMMAND;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_INPUT_VOLTAGE->text().toDouble();
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_SEND_PUMP_CONTROL_DISABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = PumpController_GeneralMSG_CMD_CTRL_OFF;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_SEND_COMMAND;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_ACTIVE_DUTYCONTROL_ON_clicked()
{
    USER_COMMAND cmd;

//    // To Daemon
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; // Active Duty Control On!
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_ACTIVE_VELOCITY_CONTROL_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
//    usleep(100*1000);

    // To PumpControl AL
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // Active Duty Control On!
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_ACTIVE_DUTYCONTROL_INIT_SPEED->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_ACTIVE_DUTYCONTROL_INIT_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PUMP_ACTIVE_CONTROL_ONOFF;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_ACTIVE_DUTYCONTROL_OFF_clicked()
{
    USER_COMMAND cmd;

//    // To Daemon
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0; // Active Duty Control Off!
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_PUMP_ACTIVE_VELOCITY_CONTROL_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
//    usleep(100*1000);

    // To PumpControl AL
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // Active Duty Control Off!
    cmd.COMMAND_DATA.USER_COMMAND = PUMP_ACTIVE_CONTROL_ONOFF;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_FINDHOME_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to all joint
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_FINDHOME_FIRST_STAGE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -2; // Command to Waist, Ankle Joints (Light Findhome Process 1st stage)
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_FINDHOME_SECOND_STAGE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -3; // Command to Rest Joints (Light Findhome Process 2nd stage)
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_PUMPCONTROL_MPC_ON_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = PUMP_ACTIVE_CONTROL_MPC_ONOFF;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_PUMPCONTROL_MPC_OFF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = PUMP_ACTIVE_CONTROL_MPC_ONOFF;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_RBTN_PS_REF_THISAL_clicked()
{
    if(ui->RBTN_PS_REF_THISAL->isChecked() == true) {
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = PUMP_CONTROL_PRESREF_SELECTION;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void PumpControlDialog::on_RBTN_PS_REF_SM_clicked()
{
    if(ui->RBTN_PS_REF_SM->isChecked() == true) {
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = PUMP_CONTROL_PRESREF_SELECTION;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
        cmd.COMMAND_TARGET = ALNum;
        pLAN->SendCommand(cmd);
    }
}

void PumpControlDialog::on_BTN_LINEARCHANGE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = PUMP_ACTIVE_CONTROL_LINEAR;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_LINEARCHANGE_DES_PRES->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_LINEARCHANGE_TIME->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_RBTN_CONST_PRES_MODE_clicked()
{
    if(ui->RBTN_CONST_PRES_MODE->isChecked() == true) {
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_SUPPLYPRES_MODE;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
        cmd.COMMAND_TARGET = RBCORE_PODO_NO;
        pLAN->SendCommand(cmd);
    }
}

void PumpControlDialog::on_RBTN_VARIABLE_PRES_MODE_clicked()
{
    if(ui->RBTN_VARIABLE_PRES_MODE->isChecked() == true) {
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_SUPPLYPRES_MODE;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
        cmd.COMMAND_TARGET = RBCORE_PODO_NO;
        pLAN->SendCommand(cmd);
    }
}

void PumpControlDialog::on_BTN_FORCESENSOR_NULLING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // Command to selected joint
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void PumpControlDialog::on_BTN_LOADCELL_OFFSET_SET_clicked()
{

}
