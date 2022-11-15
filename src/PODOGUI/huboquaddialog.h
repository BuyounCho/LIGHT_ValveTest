#ifndef HUBOQUADDIALOG_H
#define HUBOQUADDIALOG_H

#include <QDialog>
#include "RBJoystick.h"

namespace Ui {
class HuboQuadDialog;
}

class HuboQuadDialog : public QDialog
{
    Q_OBJECT

public:
    explicit HuboQuadDialog(QWidget *parent = 0);
    ~HuboQuadDialog();

private slots:
    void on_BTN_READYPOS_clicked();
    void DisplayUpdate();

    void on_BTN_WALKSTART_clicked();

    void on_BTN_WALKSTART_2_clicked();

    void on_BTN_WALKSTART_4_clicked();

    void on_BTN_WALKSTART_6_clicked();

    void on_BTN_WALKSTART_10_clicked();


    void on_BTN_LOCK_clicked();

    void on_BTN_NarrowWalk_clicked();

    void on_BTN_NarrowWalk_2_clicked();

    void on_BTN_NarrowWalk_3_clicked();


    void on_BTN_LOCK_2_clicked();


    void on_BTN_SENSOR_ENABLE_clicked();

    void on_BTN_SENSOR_FT_NULL_clicked();

    void on_BTN_NEW_IMU_ENABLE_clicked();

    void on_BTN_NEW_IMU_NULL_clicked();

    void on_BTN_NEW_IMU_RESET_clicked();

    void on_BTN_NEW_IMU_RESET_2_clicked();

    void on_BTN_JOY_CONNECT_clicked();

    void on_BTN_JOY_DISCONNECT_clicked();





    void on_BTN_W_clicked();

    void on_BTN_W_2_clicked();






    void on_BTN_PDtest_clicked();

    void on_BTN_PDtest_2_clicked();

    void on_BTN_ZERO_clicked();

    void on_BTN_WALKSTART_5_clicked();

    void on_BTN_WALKSTART_7_clicked();

    void on_BTN_WALKSTART_8_clicked();

private:
    Ui::HuboQuadDialog *ui;

    QTimer				*displayTimer;
    int             AlnumHuboQuad;
    RBJoystick      *joy;
    char            oldB[12];
    bool            isWalking;
    int             JoyGait;
};

#endif // HUBOQUADDIALOG_H
