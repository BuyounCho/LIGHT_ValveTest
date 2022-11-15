#ifndef PERFORMANCETESTDIALOG_H
#define PERFORMANCETESTDIALOG_H

#include <QDialog>

namespace Ui {
class PerformanceTestDialog;
}

class PerformanceTestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PerformanceTestDialog(QWidget *parent = 0);
    ~PerformanceTestDialog();

private slots:
    void on_PT_BTN_HOME_clicked();

    void on_PT_BTN_POSE_clicked();

    void on_PT_BTN_RPT_clicked();

    void on_PT_BTN_WALKING_clicked();

    void on_PT_BTN_CUR_ZERO_clicked();

    void on_PT_BTN_CUR_REF_clicked();

    void on_PT_BTN_CUR_MODE_clicked();

    void on_PT_BTN_COM_PEL_READY_clicked();

    void on_PT_BTN_COM_PEL_SET_clicked();

private:
    Ui::PerformanceTestDialog *ui;

    int ALNUM_TEST_AL;
};

#endif // PERFORMANCETESTDIALOG_H
