#ifndef QUADEFFTESTDIALOG_H
#define QUADEFFTESTDIALOG_H

#include <QDialog>

namespace Ui {
class QuadEffTestDialog;
}

class QuadEffTestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QuadEffTestDialog(QWidget *parent = 0);
    ~QuadEffTestDialog();

private slots:
    void UpdateDatas();

    void on_BTN_HOMEPOSE_clicked();

    void on_BTN_SQUAT_GO_clicked();

    void on_BTN_SQUAT_STOP_clicked();

    void on_BTN_SWING_GO_clicked();

    void on_BTN_SWING_STOP_clicked();

    void on_BTN_TESTJOINT_MOVEJOINT_GO_clicked();

    void on_BTN_TESTJOINT_SINEWAVE_GO_clicked();

    void on_BTN_ARMTASK_MOVEJOINT_GO_clicked();

    void on_BTN_ARMTASK_SINEWAVE_GO_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_SAVE_STOP_clicked();

private:
    Ui::QuadEffTestDialog *ui;
    int ALNum;

public:
    int Get_TESTJOINT_JointSelected();
};

#endif // QUADEFFTESTDIALOG_H
