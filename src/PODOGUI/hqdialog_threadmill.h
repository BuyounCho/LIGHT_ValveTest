#ifndef HQDIALOG_THREADMILL_H
#define HQDIALOG_THREADMILL_H

#include <QDialog>

namespace Ui {
class hqdialog_threadmill;
}

class hqdialog_threadmill : public QDialog
{
    Q_OBJECT

public:
    explicit hqdialog_threadmill(QWidget *parent = 0);
    ~hqdialog_threadmill();

private slots:
    void DisplayUpdate();

    void on_BTN_WALKSTART_clicked();

    void on_BTN_L30_clicked();

    void on_BTN_L15_clicked();

    void on_BTN_R15_clicked();

    void on_BTN_R30_clicked();

    void on_BTN_Y0_clicked();

    void on_BTN_X0_clicked();

    void on_BTN_3cm_clicked();

    void on_BTN_5cm_clicked();

    void on_BTN_10cm_clicked();

    void on_BTN_15cm_clicked();

    void on_BTN_LROT_clicked();

    void on_BTN_NOROT_clicked();

    void on_BTN_RROT_clicked();

    void on_BTN_WALKSTOP_clicked();


    void on_BTN_13cm_clicked();

    void on_BTN_R08_clicked();

    void on_BTN_L08_clicked();

    void on_BTN_18cm_clicked();

    void on_BTN_21cm_clicked();

    void on_BTN_25cm_clicked();

private:
    Ui::hqdialog_threadmill *ui;    
    QTimer				*displayTimer;
    int             AlnumHuboQuad;
    void        DO_Trot();
    void        DO_Wave();
    void        DO_Standing();
    void        DO_adjustY(double y);
    double tt_y;
    double tleft_y;
    bool yadjust;
    double maxYstep;
};

#endif // HQDIALOG_THREADMILL_H
