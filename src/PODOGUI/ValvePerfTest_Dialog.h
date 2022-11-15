#ifndef VALVEPERFTEST_DIALOG_H
#define VALVEPERFTEST_DIALOG_H

#include <QMainWindow>
#include <QTimer>
#include "qcustomplot.h"
#include <QWidget>


namespace Ui {
class ValvePerfTest_Dialog;
}

class ValvePerfTest_Dialog : public QWidget
{
    Q_OBJECT

public:
    explicit ValvePerfTest_Dialog(QWidget *parent = 0);
    ~ValvePerfTest_Dialog();

    void valve_pos_graph(QCustomPlot *customPlot);
    void flowrate_graph(QCustomPlot *customPlot);

private slots:

    void UpdateJoints();
    void realtimeDataSlot();
    void realtimeDataSlot1();

    void on_BTN_DATA_REQUEST_ENABLE_POSnVELnFORCE_clicked();

    void on_BTN_DATA_REQUEST_ENABLE_VALVEPOS_clicked();

    void on_BTN_DATA_REQUEST_ENABLE_PRESSURE_clicked();

    void on_BTN_DATA_REQUEST_ENABLE_SOMETHING_clicked();

    void on_BTN_DATA_REQUEST_DISABLE_ALL_clicked();

    void on_BTN_ENC_ZERO_clicked();

    void on_BTN_FIND_HOME_clicked();

    void on_BTN_NULLING_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_SAVE_END_clicked();

    void on_BTN_VALVE_ID_VALVE_POS_clicked();

    void on_BTN_SHOW_GRAPH1_clicked();

    void on_BTN_VALVE_ID_DEADZONE_clicked();

    void on_BTN_ASK_DEADZONE_clicked();

    void on_BTN_SHOW_DEADZONE_clicked();

    void on_BTN_VALVE_ID_FLOWRATE_clicked();

    void on_BTN_SHOW_GRAPH2_clicked();

    void on_BTN_VOLTAGEINPUT_CONSTANT_GO_clicked();

    void on_BTN_VOLTAGEINPUT_SINEWAVE_GO_clicked();

    void on_BTN_VALVEPOSITION_STEPINPUT_GO_clicked();

    void on_BTN_VALVEPOSITION_SINEWAVE_GO_clicked();

    void on_BTN_ACTPOSITION_MOVESMOOTH_GO_clicked();

    void on_BTN_ACTPOSITION_SINEWAVE_GO_clicked();

    void on_BTN_ACTPOSITION_SINEWAVE_MULTI_GO_clicked();

    void on_BTN_ACTFORCE_STEPINPUT_GO_clicked();

    void on_BTN_ACTFORCE_GRAVCOMP_GO_clicked();

    void on_BTN_ACTFORCE_SINEWAVE_GO_clicked();

    void on_BTN_ACTFORCE_GRAVCOMP_STOP_clicked();

private:
    Ui::ValvePerfTest_Dialog *ui;
    QTimer		*displayTimer;
    QString demoName;
    QTimer dataTimer;
    QCPItemTracer *itemDemoPhaseTracer;
//    QCustomPlot *customPlot;
    int currentDemoIndex;
    int ALNum;
};

#endif // VALVEPERFTEST_DIALOG_H
