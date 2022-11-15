#ifndef QUADSETTINGDIALOG_H
#define QUADSETTINGDIALOG_H

#include <QDialog>
#include <QTableWidget>

#include "CommonHeader.h"

namespace Ui {
class QuadSettingDialog;
}

class QuadSettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QuadSettingDialog(QWidget *parent = 0);
    ~QuadSettingDialog();

private slots:
    void UpdateSettings();

    void on_TW_0_itemSelectionChanged();

    void on_TW_1_itemSelectionChanged();

    void on_BTN_FOC_NULLING_clicked();

    void on_BTN_ENC_ZERO_DM_clicked();

//    void on_BTN_ENC_ZERO_LM_clicked();

    void on_CBB_DriveMode_currentIndexChanged(int index);

//    void on_TW_DriveMotorMotion_currentChanged(int index);

    void on_BTN_ENABLE_DM_clicked();

    void on_BTN_DISABLE_DM_clicked();

    void on_BTN_POS_MOV_ABS_1_clicked();

    void on_BTN_POS_MOV_ABS_2_clicked();

    void on_BTN_POS_MOV_REL_CCW_clicked();

    void on_BTN_POS_MOV_REL_CW_clicked();

    void on_BTN_POS_STOP_clicked();

    void on_CB_POS_ABS_REPETITIVE_stateChanged(int arg1);

    void on_CB_POS_REL_REPETITIVE_stateChanged(int arg1);

    void on_BTN_VEL_JOG_CCW_pressed();

    void on_BTN_VEL_JOG_CCW_released();

    void on_BTN_VEL_JOG_CW_pressed();

    void on_BTN_VEL_JOG_CW_released();

    void on_BTN_VEL_STOP_clicked();

    void on_CB_VEL_RUN_HELD_stateChanged(int arg1);

    void on_BTN_CUR_SET_CMD_1_clicked();

    void on_BTN_CUR_SET_CMD_2_clicked();

    void on_BTN_CUR_SET_CMD_3_clicked();

    void on_BTN_CUR_SET_CMD_4_clicked();

    void on_BTN_CUR_SET_CMD_5_clicked();

    void on_BTN_CUR_SET_CMD_6_clicked();

    void on_BTN_CUR_SET_CMD_7_clicked();

    void on_BTN_CUR_SET_CMD_8_clicked();

    void on_BTN_CUR_SET_CMD_9_clicked();

    void on_BTN_CUR_SET_CMD_10_clicked();

    void on_BTN_CUR_SET_CMD_ALL_clicked();

    void on_BTN_CUR_STOP_clicked();

    void on_BTN_PWM_SET_CMD_1_clicked();

    void on_BTN_PWM_SET_CMD_2_clicked();

    void on_BTN_PWM_SET_CMD_3_clicked();

    void on_BTN_PWM_SET_CMD_4_clicked();

    void on_BTN_PWM_SET_CMD_5_clicked();

    void on_BTN_PWM_SET_CMD_6_clicked();

    void on_BTN_PWM_SET_CMD_7_clicked();

    void on_BTN_PWM_SET_CMD_8_clicked();

    void on_BTN_PWM_SET_CMD_9_clicked();

    void on_BTN_PWM_SET_CMD_10_clicked();

    void on_BTN_PWM_SET_CMD_ALL_clicked();

    void on_BTN_PWM_STOP_clicked();

    /*
    void on_CBB_DriveMode_LM_currentIndexChanged(int index);

    void on_TW_LoadMotorMotion_currentChanged(int index);

    void on_BTN_ENABLE_LM_clicked();

    void on_BTN_DISABLE_LM_clicked();

    void on_BTN_VEL_JOG_CCW_LM_pressed();

    void on_BTN_VEL_JOG_CCW_LM_released();

    void on_BTN_VEL_JOG_CW_LM_pressed();

    void on_BTN_VEL_JOG_CW_LM_released();

    void on_CB_VEL_RUN_HELD_LM_stateChanged(int arg1);

    void on_BTN_VEL_STOP_LM_clicked();

    void on_BTN_CUR_SET_CMD_LM_1_clicked();

    void on_BTN_CUR_SET_CMD_LM_2_clicked();

    void on_BTN_CUR_SET_CMD_LM_3_clicked();

    void on_BTN_CUR_SET_CMD_LM_4_clicked();

    void on_BTN_CUR_SET_CMD_LM_5_clicked();

    void on_BTN_CUR_SET_CMD_LM_6_clicked();

    void on_BTN_CUR_SET_CMD_LM_7_clicked();

    void on_BTN_CUR_SET_CMD_LM_8_clicked();

    void on_BTN_CUR_SET_CMD_LM_9_clicked();

    void on_BTN_CUR_SET_CMD_LM_10_clicked();

    void on_BTN_CUR_SET_CMD_LM_ALL_clicked();

    void on_BTN_CUR_STOP_LM_clicked();

    void on_CBB_EVALTUATIOM_MODE_currentIndexChanged(int index);

    void on_BTN_EVAL_RUN_clicked();

    void on_BTN_EVAL_STOP_clicked();

    void on_BTN_UPDATE_PROTECTION_DATA_clicked();

    void on_BTN_FOC_NULLING_clicked();

    void on_BTN_TORQUE_NULLING_clicked();

    void on_BTN_REQUEST_STATE_clicked();
    */

    void on_RB_Pcon_clicked();

    void on_RB_Ccon_clicked();

    void on_RB_FOC_Pcon_clicked();

    void on_RB_FOC_Ccon_clicked();

    void on_BTN_LOGGING_DATA_clicked();

    void on_BTN_LOGGING_STOP_clicked();

    void on_BTN_SINE_REFERENCE_START_clicked();

    void on_BTN_SINE_REFERENCE_STOP_clicked();

    void on_CB_SINE_REFERENCE_RUN_HELD_stateChanged(int arg1);

    void on_BTN_CAN_CHECK_clicked();

    void on_BTN_SENSOR_CONTACT_EN_clicked();

    void on_BTN_SENSOR_CONTACT_DIS_clicked();

    void on_BTN_SENSOR_IMU_EN_clicked();

    void on_BTN_SENSOR_IMU_NULL_clicked();

    void on_BTN_SENSOR_IMU_ZERO_clicked();

    void on_BTN_SENSOR_IMU_ZERO_2_clicked();


private:
    Ui::QuadSettingDialog *ui;

    void    InitTable(QTableWidget *table, QString j_names[], int num);

    int     FindLastSelected(int tw, int row);
    void    ChangeSelectedJoint();
    void    UnselectOtherTable(int table);
    int     select_working;
    int     lastSelected;

    int ALNum;
    bool FLAG_FOC_Nulling = false;
    bool FLAG_DM_FOC_ENABLE = false;
//    bool FLAG_LM_FOC_ENABLE = false;
    int  DM_DRIVE_MODE = 0;
//    int  LM_DRIVE_MODE = 0;
    int  EVAL_MODE = 0;
//    QWidget *DMtabAddress = Q_NULLPTR;
    QWidget *DMtabAddress[5];
//    QWidget *LMtabAddress[2];
};

#endif // QUADSETTINGDIALOG_H
