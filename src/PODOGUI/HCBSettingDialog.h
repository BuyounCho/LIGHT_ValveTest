#ifndef HCBSETTINGDIALOG_H
#define HCBSETTINGDIALOG_H

#include <QWidget>
#include <QFileDialog>

namespace Ui {
class HCBSettingDialog;
}

class HCBSettingDialog : public QWidget
{
    Q_OBJECT

public:
    explicit HCBSettingDialog(QWidget *parent = 0);
    ~HCBSettingDialog();

private slots:
//    void UpdateSettings();

    void on_BTN_CAN_CHECK_clicked();

    void on_BTN_CAN_CHANNEL_ARRANGE_clicked();

    void on_BTN_PARAMETER_SET_ALL_clicked();

    void on_BTN_ENC_ENABLE_clicked();

    void on_BTN_VALVEPOS_ENABLE_clicked();

    void on_BTN_DISABLE_ALL_clicked();

    void on_BTN_ASK_EVERYTHING_clicked();

    void on_BTN_SHOW_EVERYTHING_clicked();

    void on_BTN_BOARD_TEST_ERRORRESET_clicked();

    void on_BTN_SET_BNO_clicked();

    void on_BTN_ENC_ZERO_clicked();

    void on_BTN_FINDHOME_ALL_clicked();

    void on_BTN_FINDHOME_ONEJOINT_clicked();

    void on_BTN_TORQUEFORCE_NULLING_clicked();

    void on_BTN_TORQUEFORCE_NULLING_ONE_clicked();

    void on_BTN_SOMETHING_ENABLE_clicked();

    void on_BTN_FINDHOME_FIRST_STAGE_clicked();

    void on_BTN_FINDHOME_SECOND_STAGE_clicked();

    void on_BTN_ALL_REFERENCE_RESET_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_SAVE_END_clicked();

    void on_BTN_READnSAVE_PARAMETERS_clicked();

    void on_BTN_LOADnSET_PARAMETERS_clicked();

private:
    Ui::HCBSettingDialog *ui;

    int ALNum;
    int GetSelected();
};

#endif // HCBSETTINGDIALOG_H
