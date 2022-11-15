#ifndef PUMPCONTROLDIALOG_H
#define PUMPCONTROLDIALOG_H

#include <QWidget>

namespace Ui {
class PumpControlDialog;
}

class PumpControlDialog : public QWidget
{
    Q_OBJECT

public:
    explicit PumpControlDialog(QWidget *parent = 0);
    ~PumpControlDialog();

private slots:
    void on_BTN_SEND_DUTY_clicked();

    void on_BTN_SEND_DUTY_ZERO_clicked();

    void on_BTN_SEND_PRESSURE_NULL_clicked();

private slots:
    void UpdateDatas();

    void on_BTN_ACTIVE_DUTYCONTROL_ON_clicked();

    void on_BTN_ACTIVE_DUTYCONTROL_OFF_clicked();

    void on_BTN_FINDHOME_ALL_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_SAVE_STOP_clicked();

    void on_BTN_SEND_PUMPDATA_REQUEST_ON_clicked();

    void on_BTN_SEND_PUMPDATA_REQUEST_OFF_clicked();

    void on_BTN_SEND_PUMP_CONTROL_ENABLE_clicked();

    void on_BTN_SEND_PUMP_CONTROL_DISABLE_clicked();

    void on_BTN_PUMPCONTROL_MPC_ON_clicked();

    void on_BTN_PUMPCONTROL_MPC_OFF_clicked();

    void on_RBTN_PS_REF_THISAL_clicked();

    void on_RBTN_PS_REF_SM_clicked();

    void on_BTN_LINEARCHANGE_GO_clicked();

    void on_RBTN_CONST_PRES_MODE_clicked();

    void on_RBTN_VARIABLE_PRES_MODE_clicked();

    void on_BTN_FINDHOME_FIRST_STAGE_clicked();

    void on_BTN_FINDHOME_SECOND_STAGE_clicked();

    void on_BTN_FORCESENSOR_NULLING_clicked();

    void on_BTN_LOADCELL_OFFSET_SET_clicked();

private:
    Ui::PumpControlDialog *ui;

    int ALNum;

};

#endif // PUMPCONTROLDIALOG_H
