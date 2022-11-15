#ifndef JUMPDIALOG_H
#define JUMPDIALOG_H

#include <QDialog>

namespace Ui {
class JumpDialog;
}

class JumpDialog : public QDialog
{
    Q_OBJECT

public:
    explicit JumpDialog(QWidget *parent = 0);
    ~JumpDialog();

private slots:
    void UpdateJump();

    void on_BTN_WALKSTART_clicked();

    void on_BTN_WALKSTOP_clicked();

    void on_BTN_PDTEST_clicked();

    void on_BTN_PDTESTSTOP_clicked();

    void on_BTN_SINTEST_clicked();

    void on_BTN_SINSTOP_clicked();

    void on_BTN_SINREADY_clicked();

    void on_BTN_PWM_clicked();

    void on_BTN_EXECUTE_COMMAND_clicked();

    void on_BTN_PWM_2_clicked();

    void on_BTN_PWM_3_clicked();

    void on_BTN_PWM_4_clicked();

    void on_BTN_PWM_STOP_clicked();

    void on_BTN_ENC_NULL_clicked();

    void on_BTN_CUR_NULL_clicked();

    void on_BTN_REF_ENABLE_clicked();

    void on_BTN_REF_DISABLE_clicked();

    void on_BTN_JUMPREADY_clicked();

    void on_BTN_JUMPSTART_clicked();

    void on_BTN_JUMPSTOP_clicked();

    void on_BTN_JUMP_SET_clicked();

    void on_BTN_TEST_clicked();

    void on_BTN_HOPPINGSTART_clicked();

    void on_BTN_SET_clicked();

    void on_BTN_Chnge2_Pcon_clicked();

    void on_BTN_Chnge2_CPcon_clicked();

    void on_BTN_Chnge2_CP_PODOcon_clicked();

    void on_BTN_Chnge2_Ccon_clicked();

    void on_BTN_Chnge2_CP_FFcon_clicked();

    void on_BTN_EMSTOP_clicked();

    void on_BTN_FLYREADY_clicked();

    void on_BTN_FLYSTART_clicked();

    void on_BTN_WARMUP_clicked();

    void on_BTN_StiffnessTest_clicked();

    void on_BTN_HYSTEST_clicked();

private:
    Ui::JumpDialog *ui;
    int             AlnumJUMP;
};

#endif // JUMPDIALOG_H
