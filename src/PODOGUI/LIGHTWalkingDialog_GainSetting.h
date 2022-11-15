#ifndef LIGHTWALKINGDIALOG_GAINSETTING_H
#define LIGHTWALKINGDIALOG_GAINSETTING_H

#include <QDialog>

namespace Ui {
class LIGHTWalkingDialog_GainSetting;
}

class LIGHTWalkingDialog_GainSetting : public QDialog
{
    Q_OBJECT

public:
    explicit LIGHTWalkingDialog_GainSetting(QWidget *parent = 0);
    ~LIGHTWalkingDialog_GainSetting();

private slots:
    void on_BTN_DSP_GAIN_SET_clicked();

    void on_BTN_RSSP_GAIN_SET_clicked();

    void on_BTN_LSSP_GAIN_SET_clicked();

    void on_BTN_FLOAT_GAIN_SET_clicked();

    void on_BTN_JOINT_IMPEDANCE_RF_clicked();

    void on_BTN_JOINT_IMPEDANCE_LF_clicked();

    void on_BTN_COM_LEAD_GAIN_SET_clicked();

    void on_BTN_CHECK_PARAMETERS_clicked();

    void on_CON_ANKLETORQUECOMP_GAIN_SET_clicked();

private:
    Ui::LIGHTWalkingDialog_GainSetting *ui;

    int ALNum;
};

#endif // LIGHTWALKINGDIALOG_GAINSETTING_H
