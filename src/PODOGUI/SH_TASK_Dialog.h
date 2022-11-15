#ifndef SH_TASK_DIALOG_H
#define SH_TASK_DIALOG_H

#include <QWidget>

namespace Ui {
class SH_TASK_Dialog;
}

class SH_TASK_Dialog : public QWidget
{
    Q_OBJECT

public:
    explicit SH_TASK_Dialog(QWidget *parent = 0);
    ~SH_TASK_Dialog();

private slots:
//    void UpdateSettings();

    void on_BTN_CONST_OPEN_POS_clicked();

    void on_BTN_CONST_OPEN_NEG_clicked();

    void on_BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_clicked();

    void on_BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG_clicked();

    void on_BTN_OPENLOOP_SINE_GO_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_SAVE_STOP_clicked();

    void on_BTN_LIGHT_GOTO_POSITION_clicked();

    void on_BTN_GOTO_RANDOM_POSITION_clicked();

    void on_BTN_TORQUE_CONTROL_GO_clicked();

    void on_BTN_TORQUE_CONTROL_STOP_clicked();

    void on_BTN_POSITION_CONTROL_SINEWAVE_GO_clicked();

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_clicked();

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS_clicked();

    void on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR_clicked();


    void on_LE_VARIABLE_SUPPLY_SWING_GO_clicked();

    void on_BTN_HOMEPOSE_clicked();

    void on_BTN_WALKREADY_clicked();

private:
    Ui::SH_TASK_Dialog *ui;

    int ALNum;
    int Get_BNO_Selected();
};

#endif // SH_TASK_DIALOG_H
