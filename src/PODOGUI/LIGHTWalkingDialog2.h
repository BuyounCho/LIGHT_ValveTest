#ifndef LIGHTWALKINGDIALOG2_H
#define LIGHTWALKINGDIALOG2_H

#include "qdialog.h"
#include "CommonHeader.h"
#include "JoyStick/joystickclass.h"
#include "JoyStick/joystickvariable.h"

namespace Ui {
class LIGHTWalkingDialog2;
}

class LIGHTWalkingDialog2 : public QDialog
{
    Q_OBJECT

public:
    explicit LIGHTWalkingDialog2(QWidget *parent = 0);
    ~LIGHTWalkingDialog2();

private slots:

    // IMU
    void UpdateIMUData();
    void on_BTN_IMU_ENABLE_clicked();
    void on_BTN_IMU_DISABLE_clicked();
    void on_BTN_IMU_NULL_clicked();

    // JoyStick
    void UpdateJoyStickData();
    void GetJoystickData(void);
    void InitJoystickData(void);
    void CheckClickJoystickData(bool _Act, bool* _pFlag_Clicked);

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_clicked();

    void on_RBTN_OPERATIONMODE_CHOREONOID_toggled(bool checked);

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS_clicked();

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR_clicked();

    void on_BTN_COM_SMOOTH_MOTION_GO_clicked();

    void on_BTN_RFSWINGUP_GO_clicked();

    void on_BTN_RFSWINGDOWN_GO_clicked();

    void on_BTN_DYNAMICWALKING_GO_clicked();

    void on_BTN_COM_SMOOTH_MOTION_GO_LDSP_clicked();

    void on_BTN_LFSWINGUP_GO_clicked();

    void on_BTN_LFSWINGDOWN_GO_clicked();

    void on_BTN_JOYSTICK_ENABLE_clicked();

    void on_BTN_JOYSTICK_DISABLE_clicked();

    void on_BTN_DYNAMICWALKING_JOYSTICK_GO_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_SAVE_END_clicked();

    void on_BTN_FULLTASK_GO_clicked();

    void on_BTN_IMU_FIND_OFFSET_clicked();

    void on_CBX_FULLTASK_UPDATE_PUMPREF_toggled(bool checked);

    void on_BTN_JUMPTEST_GO_clicked();

private:
    Ui::LIGHTWalkingDialog2 *ui;
    QTimer  *displayTimer;

    QPalette *palette;

    RBJoystick          *JoyStick;
    bool    Flag_JoyStick_OnOff;

    int     JOY_LJOG_RL, JOY_LJOG_UD;
    int     JOY_RJOG_RL, JOY_RJOG_UD;
    int     JOY_CROSS_RL, JOY_CROSS_UD;

    char    JOY_A, JOY_B, JOY_X, JOY_Y;
    char    JOY_LB, JOY_LT, JOY_RB, JOY_RT;
    char    JOY_BACK, JOY_START;
    char    JOY_LJOG_BTN, JOY_RJOG_BTN;

    int ALNum;
};

#endif // LIGHTWALKINGDIALOG2_H
