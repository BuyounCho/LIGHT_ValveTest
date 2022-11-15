#include "LIGHTWalkingDialog.h"
#include "ui_LIGHTWalkingDialog.h"
#include "../../src/ALPrograms/LIGHTWalking/LIGHT_commands.h"

#include "BasicFiles/PODOALDialog.h"
#include <iostream>
using namespace std;

LIGHTWalkingDialog::LIGHTWalkingDialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LIGHTWalkingDialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("LIGHTWalking");
    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateIMUData()));
}

LIGHTWalkingDialog::~LIGHTWalkingDialog()
{
    delete ui;
}

void LIGHTWalkingDialog::UpdateIMUData(){
    ui->LE_IMU_ROLL->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Roll*R2D));
    ui->LE_IMU_PITCH->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Pitch*R2D));
    ui->LE_IMU_YAW->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Yaw*R2D));

    if(PODO_DATA.CoreREF.Simulation_DataEnable == 0) { // Data Receiving from Robot
        ui->LE_ROBOT_OR_SIM->setText(QString().sprintf("%s", "Robot"));
        QPalette *palette = new QPalette();
        palette->setColor(QPalette::Text,Qt::red);
        ui->LE_ROBOT_OR_SIM->setPalette(*palette);
    } else if (PODO_DATA.CoreREF.Simulation_DataEnable == 1) {  // Data Receiving from Simulation
        ui->LE_ROBOT_OR_SIM->setText(QString().sprintf("%s", "Simulation"));
        QPalette *palette = new QPalette();
        palette->setColor(QPalette::Text,Qt::blue);
        ui->LE_ROBOT_OR_SIM->setPalette(*palette);
    }
}

void LIGHTWalkingDialog::on_BTN_IMU_ENABLE_clicked()
{
    // new imu enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_IMU_DISABLE_clicked()
{
    // new imu disable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_IMU_NULL_clicked()
{
    // new imu nulling
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_IMU_FIND_OFFSET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_JOINTSPACE_MOVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_GOTO_HOMEPOSE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_WORKSPACE_MOVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_WORKSPACE_RIGHT_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_WORKSPACE_RIGHT_Y->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_WORKSPACE_RIGHT_Z->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_WORKSPACE_LEFT_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_WORKSPACE_LEFT_Y->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_WORKSPACE_LEFT_Z->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_WORKSPACE_TIME->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_WORKSPACE_MOVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_RBTN_OPERATIONMODE_CHOREONOID_toggled(bool checked)
{
    USER_COMMAND cmd;

    if(ui->RBTN_OPERATIONMODE_ROBOT->isChecked()) cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    else if(ui->RBTN_OPERATIONMODE_CHOREONOID->isChecked()) cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SIMLATION_MODE_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->CBX_ROBOT_CONTROLMETHOD_RHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->CBX_ROBOT_CONTROLMETHOD_RHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->CBX_ROBOT_CONTROLMETHOD_RHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = ui->CBX_ROBOT_CONTROLMETHOD_RKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[4] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[5] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[6] = ui->CBX_ROBOT_CONTROLMETHOD_LHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[7] = ui->CBX_ROBOT_CONTROLMETHOD_LHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[8] = ui->CBX_ROBOT_CONTROLMETHOD_LHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[9] = ui->CBX_ROBOT_CONTROLMETHOD_LKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[10] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[11] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_POSorTOR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS_clicked()
{
    ui->CBX_ROBOT_CONTROLMETHOD_RHR->setChecked(0);
//    ui->CBX_ROBOT_CONTROLMETHOD_RHY->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RHP->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RKN->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RANK->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LHR->setChecked(0);
//    ui->CBX_ROBOT_CONTROLMETHOD_LHY->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LHP->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LKN->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LANK->setChecked(0);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->CBX_ROBOT_CONTROLMETHOD_RHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->CBX_ROBOT_CONTROLMETHOD_RHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->CBX_ROBOT_CONTROLMETHOD_RHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = ui->CBX_ROBOT_CONTROLMETHOD_RKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[4] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[5] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[6] = ui->CBX_ROBOT_CONTROLMETHOD_LHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[7] = ui->CBX_ROBOT_CONTROLMETHOD_LHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[8] = ui->CBX_ROBOT_CONTROLMETHOD_LHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[9] = ui->CBX_ROBOT_CONTROLMETHOD_LKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[10] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[11] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_POSorTOR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR_clicked()
{
    ui->CBX_ROBOT_CONTROLMETHOD_RHR->setChecked(1);
//    ui->CBX_ROBOT_CONTROLMETHOD_RHY->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RHP->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RKN->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RANK->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LHR->setChecked(1);
//    ui->CBX_ROBOT_CONTROLMETHOD_LHY->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LHP->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LKN->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LANK->setChecked(1);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->CBX_ROBOT_CONTROLMETHOD_RHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->CBX_ROBOT_CONTROLMETHOD_RHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->CBX_ROBOT_CONTROLMETHOD_RHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = ui->CBX_ROBOT_CONTROLMETHOD_RKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[4] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[5] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[6] = ui->CBX_ROBOT_CONTROLMETHOD_LHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[7] = ui->CBX_ROBOT_CONTROLMETHOD_LHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[8] = ui->CBX_ROBOT_CONTROLMETHOD_LHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[9] = ui->CBX_ROBOT_CONTROLMETHOD_LKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[10] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[11] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_POSorTOR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void LIGHTWalkingDialog::on_BTN_COMMOVE_GO_clicked()
{

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_COMMOVE_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_COMMOVE_Y->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_COMMOVE_Z->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_COMMOVE_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_COMMOVE_PELVIS_PITCH->text().toDouble();

    if(ui->RBTN_COM_REFFRAME_RF->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;
    } else if(ui->RBTN_COM_REFFRAME_LF->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    } else {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    }

    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_COM_MOVING_DSP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);

}

void LIGHTWalkingDialog::on_BTN_AIRWALKING_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_AIRWALKING_STRIDE->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_AIRWALKING_STEPTIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->CON_AIRWALKING_NUM->text().toInt();

    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_AIRWALKING;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_COMMOVE_SSP_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_COMMOVE_SSP_SWING_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_COMMOVE_SSP_SWING_Y->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_COMMOVE_SSP_SWING_Z->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_COMMOVE_SSP_TIME->text().toDouble();

    if(ui->RBTN_COM_SSP_REFFRAME_RF->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;
    } else if(ui->RBTN_COM_SSP_REFFRAME_LF->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    } else {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    }

    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_COM_MOVING_SSP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_RBTN_COM_SSP_REFFRAME_RF_clicked()
{
    ui->RBTN_COM_SSP_REFFRAME_RF->setChecked(true);
    ui->RBTN_COM_SSP_REFFRAME_LF->setChecked(false);

    ui->CON_COMMOVE_SSP_SWING_X->setText("0.00");
    ui->CON_COMMOVE_SSP_SWING_Y->setText("0.31");
    ui->CON_COMMOVE_SSP_SWING_Z->setText("0.05");
    ui->CON_COMMOVE_SSP_TIME->setText("3.0");
}
void LIGHTWalkingDialog::on_RBTN_COM_SSP_REFFRAME_LF_clicked()
{
    ui->RBTN_COM_SSP_REFFRAME_RF->setChecked(false);
    ui->RBTN_COM_SSP_REFFRAME_LF->setChecked(true);

    ui->CON_COMMOVE_SSP_SWING_X->setText("0.00");
    ui->CON_COMMOVE_SSP_SWING_Y->setText("-0.31");
    ui->CON_COMMOVE_SSP_SWING_Z->setText("0.05");
    ui->CON_COMMOVE_SSP_TIME->setText("3.0");
}

void LIGHTWalkingDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = true; // save flag on!
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_DATA_SAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SAVE_END_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = false; // save flag off
    string filename = ui->CON_SAVE_FILENAME->text().toStdString(); // filename string
    if(filename.empty()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = NULL;
    } else {
        strcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, filename.c_str());
    }
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_DATA_SAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_DSP_RF_X_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0; // DSP_RF_X
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_DSP_RF_Y_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1; // DSP_RF_Y
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_DSP_LF_X_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2; // DSP_LF_X
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_DSP_LF_Y_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 3; // DSP_LF_Y
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_SSP_RF_X_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 4; // SSP_RF_X
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_SSP_RF_Y_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 5; // SSP_RF_Y
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_SSP_LF_X_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 6; // SSP_LF_X
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SYSID_COMREF2ZMP_SSP_LF_Y_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SYSID_COM;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_SYSID_COMREF2ZMP_COMMAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_SYSID_COMREF2ZMP_NUM->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_SYSID_COMREF2ZMP_DFREQ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 7; // SSP_LF_Y
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}


void LIGHTWalkingDialog::on_BTN_SUPPORT_TRANSITION_RDSP2RSSP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SUPPORT_TRANSITION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; // RDSP2RSSP
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SUPPORT_TRANSITION_RSSP2RDSP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SUPPORT_TRANSITION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2; // RSSP2RDSP
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SUPPORT_TRANSITION_RDSP2LDSP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SUPPORT_TRANSITION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3; // RDSP2LDSP
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SUPPORT_TRANSITION_LDSP2RDSP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SUPPORT_TRANSITION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 4; // LDSP2RDSP
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SUPPORT_TRANSITION_LDSP2LSSP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SUPPORT_TRANSITION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 5; // LDSP2LSSP
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SUPPORT_TRANSITION_LSSP2LDSP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SUPPORT_TRANSITION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 6; // LSSP2LDSP
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_KINEMATIC_COMPENSATION_ON_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = KINEMATIC_COMPENSATION_ONOFF; // parameter type setting
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = true;
//    cmd.COMMAND_TARGET = ALNum;
//    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_KINEMATIC_COMPENSATION_OFF_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = KINEMATIC_COMPENSATION_ONOFF; // parameter type setting
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = false;
//    cmd.COMMAND_TARGET = ALNum;
//    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_SQUAT_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SQUAT;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] =  ui->CON_SQUAT_HEIGHT->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] =  ui->CON_SQUAT_PERIOD->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] =  ui->CON_SQUAT_NUMBER->text().toInt();

    if(ui->RBTN_SQUAT_MODE_DSP->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    } else if(ui->RBTN_SQUAT_MODE_SSP->isChecked()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    }

    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_RBTN_SQUAT_MODE_DSP_clicked()
{
    ui->CON_SQUAT_HEIGHT->setText("0.12");
    ui->CON_SQUAT_PERIOD->setText("2.50");
    ui->CON_SQUAT_NUMBER->setText("5");
}

void LIGHTWalkingDialog::on_RBTN_SQUAT_MODE_SSP_clicked()
{
    ui->CON_SQUAT_HEIGHT->setText("0.12");
    ui->CON_SQUAT_PERIOD->setText("3.80");
    ui->CON_SQUAT_NUMBER->setText("5");
}

void LIGHTWalkingDialog::on_BTN_ANKLETORQUECOMP_ON_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_ANKLETORQUECOMP_ONOFF;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = true;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog::on_BTN_ANKLETORQUECOMP_OFF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_ANKLETORQUECOMP_ONOFF;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = false;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}
