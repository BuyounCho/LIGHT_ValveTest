#ifndef JOINTDIALOG_H
#define JOINTDIALOG_H

#include <QDialog>
#include "CommonHeader.h"

namespace Ui {
class JointDialog;
}

class JointDialog : public QDialog
{
    Q_OBJECT

public:
    explicit JointDialog(QWidget *parent = 0);
    ~JointDialog();

private slots:
    void UpdateJoints();

    void GraphInitialize();
    void GraphUpdate_RightLeg();
    void GraphUpdate_IMU();
    void GraphUpdate_Pump();

private:
    Ui::JointDialog *ui;
    QTimer  *readTimer;
    int     Freq_GraphUpdate;

    double temp_angle[12][300];
    double temp_refangle[12][300];

    double temp_IMU[6][150];

    QVector<double> temp_time;
    QVector<double> temp_time_prev;
    QVector<double> temp_PumpPressureRef;
    QVector<double> temp_PumpPressureRef_prev;
    QVector<double> temp_PumpPressure;
    QVector<double> temp_PumpFlowrateRef;
    QVector<double> temp_PumpFlowrate;

};

#endif // JOINTDIALOG_H
