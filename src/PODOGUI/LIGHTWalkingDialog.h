#ifndef LIGHTWALKINGDIALOG_H
#define LIGHTWALKINGDIALOG_H

#include <QWidget>
#include "CommonHeader.h"

namespace Ui {
class LIGHTWalkingDialog;
}

class LIGHTWalkingDialog : public QWidget
{
    Q_OBJECT

public:
    explicit LIGHTWalkingDialog(QWidget *parent = 0);
    ~LIGHTWalkingDialog();

private slots:
    void UpdateIMUData();

    void on_BTN_JOINTSPACE_MOVE_GO_clicked();

    void on_BTN_WORKSPACE_MOVE_GO_clicked();

    void on_RBTN_OPERATIONMODE_CHOREONOID_toggled(bool checked);

    void on_BTN_COMMOVE_GO_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_SAVE_END_clicked();

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_clicked();

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS_clicked();

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR_clicked();

    void on_BTN_IMU_ENABLE_clicked();

    void on_BTN_IMU_DISABLE_clicked();

    void on_BTN_IMU_NULL_clicked();

    void on_BTN_SYSID_COMREF2ZMP_DSP_RF_X_clicked();

    void on_BTN_SYSID_COMREF2ZMP_DSP_RF_Y_clicked();

    void on_BTN_SYSID_COMREF2ZMP_DSP_LF_X_clicked();

    void on_BTN_SYSID_COMREF2ZMP_DSP_LF_Y_clicked();

    void on_BTN_SUPPORT_TRANSITION_RDSP2RSSP_clicked();

    void on_BTN_SUPPORT_TRANSITION_RSSP2RDSP_clicked();

    void on_BTN_SUPPORT_TRANSITION_RDSP2LDSP_clicked();

    void on_BTN_SUPPORT_TRANSITION_LDSP2RDSP_clicked();

    void on_BTN_SUPPORT_TRANSITION_LDSP2LSSP_clicked();

    void on_BTN_SUPPORT_TRANSITION_LSSP2LDSP_clicked();

    void on_BTN_COMMOVE_SSP_GO_clicked();

    void on_RBTN_COM_SSP_REFFRAME_LF_clicked();

    void on_RBTN_COM_SSP_REFFRAME_RF_clicked();

    void on_BTN_SYSID_COMREF2ZMP_SSP_LF_X_clicked();

    void on_BTN_SYSID_COMREF2ZMP_SSP_LF_Y_clicked();

    void on_BTN_SYSID_COMREF2ZMP_SSP_RF_X_clicked();

    void on_BTN_SYSID_COMREF2ZMP_SSP_RF_Y_clicked();

    void on_BTN_KINEMATIC_COMPENSATION_ON_clicked();

    void on_BTN_KINEMATIC_COMPENSATION_OFF_clicked();

    void on_BTN_SQUAT_GO_clicked();

    void on_RBTN_SQUAT_MODE_DSP_clicked();

    void on_RBTN_SQUAT_MODE_SSP_clicked();

    void on_BTN_AIRWALKING_GO_clicked();

    void on_BTN_IMU_FIND_OFFSET_clicked();

    void on_BTN_ANKLETORQUECOMP_ON_clicked();

    void on_BTN_ANKLETORQUECOMP_OFF_clicked();

private:
    Ui::LIGHTWalkingDialog *ui;
//    int GetSelected_Robot_ControlMethod();
    int ALNum;
};


#endif // LIGHTWALKINGDIALOG_H
