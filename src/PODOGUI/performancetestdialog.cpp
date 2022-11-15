#include "performancetestdialog.h"
#include "ui_performancetestdialog.h"

#include "BasicFiles/PODOALDialog.h"

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
    PERFORMANCE_TEST_MOTION,
    PERFORMANCE_TEST_CURRENT,
    PERFORMANCE_TEST_COM_PELVIS_READY,
    PERFORMANCE_TEST_COM_PELVIS_MOTION
};


PerformanceTestDialog::PerformanceTestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PerformanceTestDialog)
{
    ui->setupUi(this);

    ALNUM_TEST_AL = PODOALDialog::GetALNumFromFileName("Quadruped");
}

PerformanceTestDialog::~PerformanceTestDialog()
{
    delete ui;
}

void PerformanceTestDialog::on_PT_BTN_HOME_clicked()
{
    // home
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_MOTION;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);
}

void PerformanceTestDialog::on_PT_BTN_POSE_clicked()
{
    // pose
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_MOTION;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);
}

void PerformanceTestDialog::on_PT_BTN_RPT_clicked()
{
    // repeat
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->PT_LEDIT_TIME->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->PT_LEDIT_NUM->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->PT_LEDIT_SCALE->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_MOTION;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);


}

void PerformanceTestDialog::on_PT_BTN_WALKING_clicked()
{
    // ho walk
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->PT_LEDIT_TIME->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->PT_LEDIT_NUM->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->PT_LEDIT_SCALE->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_MOTION;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);
}

void PerformanceTestDialog::on_PT_BTN_CUR_ZERO_clicked()
{
    int ref_bno = (int)ui->PT_LEDIT_CUR_BNO->text().toInt();
    double ref_cur = (double)ui->PT_LEDIT_CUR_REF->text().toDouble();
    // cur 0
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ref_bno;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_CURRENT;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);
}

void PerformanceTestDialog::on_PT_BTN_CUR_REF_clicked()
{
    // cur ref
    int ref_bno = (int)ui->PT_LEDIT_CUR_BNO->text().toInt();
    double ref_cur = (double)ui->PT_LEDIT_CUR_REF->text().toDouble();
    // cur 0
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ref_bno;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ref_cur;
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_CURRENT;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);
}

void PerformanceTestDialog::on_PT_BTN_CUR_MODE_clicked()
{
    // cur mode

    int ref_bno = (int)ui->PT_LEDIT_CUR_BNO->text().toInt();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ref_bno;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = 0;
    pLAN->SendCommand(cmd);
}


void PerformanceTestDialog::on_PT_BTN_COM_PEL_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = (double)ui->PT_LEDIT_COMZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = (double)ui->PT_LEDIT_DURATION->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_COM_PELVIS_READY;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);
}

void PerformanceTestDialog::on_PT_BTN_COM_PEL_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = (double)ui->PT_LEDIT_COMX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = (double)ui->PT_LEDIT_COMY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = (double)ui->PT_LEDIT_COMZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = (double)ui->PT_LEDIT_PEL_ROLL->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = (double)ui->PT_LEDIT_PEL_PITCH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = (double)ui->PT_LEDIT_PEL_YAW->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = (double)ui->PT_LEDIT_DURATION->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = PERFORMANCE_TEST_COM_PELVIS_MOTION;
    cmd.COMMAND_TARGET = ALNUM_TEST_AL;
    pLAN->SendCommand(cmd);
}


