#include "LIGHTWalkingDialog_GainSetting.h"
#include "ui_LIGHTWalkingDialog_GainSetting.h"
#include "../../src/ALPrograms/LIGHTWalking/LIGHT_commands.h"

#include "BasicFiles/PODOALDialog.h"
#include <iostream>
using namespace std;

LIGHTWalkingDialog_GainSetting::LIGHTWalkingDialog_GainSetting(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LIGHTWalkingDialog_GainSetting)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("LIGHTWalking");
}

LIGHTWalkingDialog_GainSetting::~LIGHTWalkingDialog_GainSetting()
{
    delete ui;
}

void LIGHTWalkingDialog_GainSetting::on_BTN_DSP_GAIN_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = SUPPORTCONTROL_DSP; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_DSP_GAIN_BODY_POS_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_DSP_GAIN_BODY_ORI_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_DSP_GAIN_FOOT_POS_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_DSP_GAIN_FOOT_ORI_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_DSP_GAIN_BODY_POS_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_DSP_GAIN_BODY_ORI_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_DSP_GAIN_FOOT_POS_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->CON_DSP_GAIN_FOOT_ORI_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->CON_DSP_GAIN_WEIGHT_COMP->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_BTN_RSSP_GAIN_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = SUPPORTCONTROL_RSSP; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_RSSP_GAIN_BODY_POS_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_RSSP_GAIN_BODY_ORI_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_RSSP_GAIN_FOOT_POS_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_RSSP_GAIN_FOOT_ORI_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_RSSP_GAIN_BODY_POS_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_RSSP_GAIN_BODY_ORI_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_RSSP_GAIN_FOOT_POS_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->CON_RSSP_GAIN_FOOT_ORI_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->CON_RSSP_GAIN_WEIGHT_COMP->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_BTN_LSSP_GAIN_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = SUPPORTCONTROL_LSSP; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_LSSP_GAIN_BODY_POS_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_LSSP_GAIN_BODY_ORI_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_LSSP_GAIN_FOOT_POS_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_LSSP_GAIN_FOOT_ORI_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_LSSP_GAIN_BODY_POS_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_LSSP_GAIN_BODY_ORI_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_LSSP_GAIN_FOOT_POS_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->CON_LSSP_GAIN_FOOT_ORI_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->CON_LSSP_GAIN_WEIGHT_COMP->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_BTN_FLOAT_GAIN_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = SUPPORTCONTROL_FLOAT; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_FLOAT_GAIN_FOOT_POS_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_FLOAT_GAIN_FOOT_ORI_STIFF->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_FLOAT_GAIN_FOOT_POS_DAMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_FLOAT_GAIN_FOOT_ORI_DAMP->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_BTN_JOINT_IMPEDANCE_RF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = JOINGIMPEDANCE_RF; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_RF_ONCONTACT_POS_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_RF_ONCONTACT_POS_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_RF_ONCONTACT_POS_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_RF_ONCONTACT_ORI_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_RF_ONCONTACT_ORI_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_RF_ONCONTACT_ORI_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_RF_ONCONTACT_POS_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->CON_RF_ONCONTACT_POS_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->CON_RF_ONCONTACT_POS_DZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->CON_RF_ONCONTACT_ORI_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->CON_RF_ONCONTACT_ORI_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->CON_RF_ONCONTACT_ORI_DZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[12] = ui->CON_RF_OFFCONTACT_POS_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[13] = ui->CON_RF_OFFCONTACT_POS_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[14] = ui->CON_RF_OFFCONTACT_POS_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[15] = ui->CON_RF_OFFCONTACT_ORI_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[16] = ui->CON_RF_OFFCONTACT_ORI_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[17] = ui->CON_RF_OFFCONTACT_ORI_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[18] = ui->CON_RF_OFFCONTACT_POS_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[19] = ui->CON_RF_OFFCONTACT_POS_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[20] = ui->CON_RF_OFFCONTACT_POS_DZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[21] = ui->CON_RF_OFFCONTACT_ORI_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[22] = ui->CON_RF_OFFCONTACT_ORI_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[23] = ui->CON_RF_OFFCONTACT_ORI_DZ->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_BTN_JOINT_IMPEDANCE_LF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = JOINGIMPEDANCE_LF; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_LF_ONCONTACT_POS_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_LF_ONCONTACT_POS_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_LF_ONCONTACT_POS_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_LF_ONCONTACT_ORI_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_LF_ONCONTACT_ORI_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_LF_ONCONTACT_ORI_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_LF_ONCONTACT_POS_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->CON_LF_ONCONTACT_POS_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->CON_LF_ONCONTACT_POS_DZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->CON_LF_ONCONTACT_ORI_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->CON_LF_ONCONTACT_ORI_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->CON_LF_ONCONTACT_ORI_DZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[12] = ui->CON_LF_OFFCONTACT_POS_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[13] = ui->CON_LF_OFFCONTACT_POS_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[14] = ui->CON_LF_OFFCONTACT_POS_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[15] = ui->CON_LF_OFFCONTACT_ORI_KX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[16] = ui->CON_LF_OFFCONTACT_ORI_KY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[17] = ui->CON_LF_OFFCONTACT_ORI_KZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[18] = ui->CON_LF_OFFCONTACT_POS_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[19] = ui->CON_LF_OFFCONTACT_POS_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[20] = ui->CON_LF_OFFCONTACT_POS_DZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[21] = ui->CON_LF_OFFCONTACT_ORI_DX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[22] = ui->CON_LF_OFFCONTACT_ORI_DY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[23] = ui->CON_LF_OFFCONTACT_ORI_DZ->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_BTN_COM_LEAD_GAIN_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = CoMLEADCOMPENSATE; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_COM_LEAD_WN_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_COM_LEAD_WN_Y->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_COM_LEAD_ZETA_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_COM_LEAD_ZETA_Y->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_BTN_CHECK_PARAMETERS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = CHECK_PARAMETERS; // parameter type setting
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog_GainSetting::on_CON_ANKLETORQUECOMP_GAIN_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_PARAMETER_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ANKLETORQUECOMPENSATE; // parameter type setting
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_ANKLETORQUECOMP_KP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_ANKLETORQUECOMP_KI->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}
