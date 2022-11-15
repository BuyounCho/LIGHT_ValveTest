#include "ValvePerfTest_Dialog.h"
#include "ui_ValvePerfTest_Dialog.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QMessageBox>
#include <qvector2d.h>
#include "qcustomplot.h"

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>
#include <QTimer>

using namespace std;

int data_num = 0;



ValvePerfTest_Dialog::ValvePerfTest_Dialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ValvePerfTest_Dialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("ValvePerfTest");

    //connect(displayTimer, SIGNAL(timeout()), this, SLOT(UpdateSettings()));
    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateJoints()));

    displayTimer = new QTimer();

    setGeometry(400, 250, 600, 450);

    valve_pos_graph(ui->customPlot);
    flowrate_graph(ui->customPlot2);
    //realtime_graph(ui->customPlot3);

}

ValvePerfTest_Dialog::~ValvePerfTest_Dialog()
{
    delete ui;
}

inline void DisplayActPos(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentActPos));
}
inline void DisplayActVel(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentActVel));
}
inline void DisplayActForce(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentActForce));
}
inline void DisplayValvePos(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentValvePos));
}
inline void DisplayValvePosRef(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentValvePosRef));
}
inline void DisplayPWM(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentPWM));
}
inline void DisplayPresA(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentPressureA));
}
inline void DisplayPresB(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentPressureB));
}
inline void DisplayData1(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentData1));
}
inline void DisplayData2(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentData2));
}
inline void DisplayData3(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentData3));
}
inline void DisplayData4(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentData4));
}

void ValvePerfTest_Dialog::UpdateJoints(){
    int BNO = ui->LE_BNO->text().toInt(); //BNO
    DisplayActPos(BNO, ui->LE_ACT_POS);
    DisplayActVel(BNO, ui->LE_ACT_VEL);
    DisplayActForce(BNO, ui->LE_ACT_FORCE);
    DisplayValvePos(BNO , ui->LE_VALVEPOS);
    DisplayValvePosRef(BNO, ui->LE_VALVE_POS_REF);
    DisplayPWM(BNO, ui->LE_PWM);
    DisplayPresA(BNO, ui->LE_PRES_A);
    DisplayPresB(BNO, ui->LE_PRES_B);
    DisplayData1(BNO, ui->LE_DATA1);
    DisplayData2(BNO, ui->LE_DATA2);
    DisplayData3(BNO, ui->LE_DATA3);
    DisplayData4(BNO, ui->LE_DATA4);
}

void ValvePerfTest_Dialog::valve_pos_graph(QCustomPlot *customPlot)
{
  customPlot->addGraph(0);
  customPlot->xAxis->setLabel("PWM");
  customPlot->yAxis->setLabel("VALVE_POS");
  customPlot->graph(0)->setPen(QPen(QColor(40, 110, 255)));
  customPlot->xAxis->setRange(-9999, 9999);
  customPlot->axisRect()->setupFullAxesBox();
  customPlot->yAxis->setRange(700,3800);

  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
  dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void ValvePerfTest_Dialog::flowrate_graph(QCustomPlot *customPlot)
{
  customPlot->addGraph();
  customPlot->xAxis->setLabel("VALVE_POS [-100% ~ 100%]");
  customPlot->yAxis->setLabel("FLOWRATE");
  customPlot->graph(0)->setPen(QPen(QColor(110, 10, 255)));
  customPlot->xAxis->setRange(-100, 100);
  customPlot->axisRect()->setupFullAxesBox();
  customPlot->yAxis->setRange(-8, 8);

  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot1()));
  dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void ValvePerfTest_Dialog::realtimeDataSlot() // ValvePos ID result graph
{
    int BNO = ui->LE_BNO->text().toInt(); //BNO
    ui->customPlot->graph(0)->data()->clear();
    data_num = 41;
    for(int graph_index=0;graph_index<data_num;graph_index++)
    {
        ui->customPlot->graph(0)->addData(PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_PWM_VALVE_ID[graph_index], PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_POS_VALVE_ID[graph_index]);
    }
    ui->customPlot->replot();

}

void ValvePerfTest_Dialog::realtimeDataSlot1() // Flowrate ID result graph
{
    int BNO = ui->LE_BNO->text().toInt(); //BNO
    ui->customPlot2->graph(0)->data()->clear();
    data_num = 51;
    for(int graph_index=0;graph_index<data_num;graph_index++)
    {
        ui->customPlot2->graph(0)->addData(PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_POSITION_FLOWRATE_ID[graph_index],
                                           PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_FLOWRATE_FLOWRATE_ID[graph_index]);
    }
    ui->customPlot2->replot();
}

// ==============================================================================================

void ValvePerfTest_Dialog::on_BTN_DATA_REQUEST_ENABLE_POSnVELnFORCE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // Enable Requesting Signal
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0; // Requested Data Type : Pos & Vel & For
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_DATA_REQUEST_ENABLE_VALVEPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // Enable Requesting Signal
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1; // Requested Data Type : Valve Pos & PWM
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_DATA_REQUEST_ENABLE_PRESSURE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // Enable Requesting Signal
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 2; // Requested Data Type : Pressure
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_DATA_REQUEST_ENABLE_SOMETHING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // Enable Requesting Signal
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 3; // Requested Data Type : Other Information
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_DATA_REQUEST_DISABLE_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0; // Disable Requesting Signal
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -1; // Requested Data Type : ALL
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_REQUEST_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ENC_ZERO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_ENC_ZERO;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_FIND_HOME_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_FINDHOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_NULLING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; // Save Start!
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_SAVE_END_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0; // Save Stop!
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_VALVE_ID_VALVE_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_VOLTAGE2VALVEPOS_ID;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_SHOW_GRAPH1_clicked()
{
    if(ui->RBTN_VALVEPOS_PWM_GRAPH_DDV->isChecked()) {
        ui->customPlot->xAxis->setRange(-9999, 9999);
        ui->customPlot->yAxis->setRange(700,3800);
    } else if(ui->RBTN_VALVEPOS_PWM_GRAPH_MOOG->isChecked()) {
        ui->customPlot->xAxis->setRange(-7000, 7000);
        ui->customPlot->yAxis->setRange(-10500,10500);
    } else if(ui->RBTN_VALVEPOS_PWM_GRAPH_KNR->isChecked()) {
        ui->customPlot->xAxis->setRange(-7000, 7000);
        ui->customPlot->yAxis->setRange(-30500,30500);
    }
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_VOLTAGE2VALVEPOS_ASKRESULT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_VALVE_ID_DEADZONE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_DEADZONE_ID;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ASK_DEADZONE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_DEADZONE_ASKRESULT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_SHOW_DEADZONE_clicked()
{
    unsigned int _BNO = ui->LE_BNO->text().toInt();
    ui->LE_VALVE_DEADZONE_MINUS->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[_BNO][0].HCB_Info.VALVE_DZ_MINUS));
    ui->LE_VALVE_DEADZONE_PLUS->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[_BNO][0].HCB_Info.VALVE_DZ_PLUS));
    ui->LE_VALVE_DEADZONE_CENTER->setText(QString().sprintf("%d", PODO_DATA.CoreSEN.ENCODER[_BNO][0].HCB_Info.VALVE_CENTER_POS));
}

void ValvePerfTest_Dialog::on_BTN_VALVE_ID_FLOWRATE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_VALVEPOS2FLOWRATE_ID;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_SHOW_GRAPH2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); // Target Board
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_VALVEPERFTEST_VALVEPOS2FLOWRATE_ASKRESULT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_VOLTAGEINPUT_CONSTANT_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_VOLTAGEINPUT_CONSTANT_VALUE->text().toInt(); // mV
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_VALVEVOLTAGE_STEP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_VOLTAGEINPUT_SINEWAVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_VOLTAGEINPUT_SINEWAVE_MAG->text().toInt(); // mV
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->LE_VOLTAGEINPUT_SINEWAVE_NUM->text().toInt(); // times
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_VOLTAGEINPUT_SINEWAVE_PER->text().toDouble(); // sec
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_VALVEVOLTAGE_SINEWAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_VALVEPOSITION_STEPINPUT_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_VALVEPOSITION_STEPINPUT_VALUE->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_VALVEPOSITION_STEP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_VALVEPOSITION_SINEWAVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_VALVEPOSITION_SINEWAVE_MAG->text().toInt(); // mV
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->LE_VALVEPOSITION_SINEWAVE_NUM->text().toInt(); // times
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_VALVEPOSITION_SINEWAVE_PER->text().toDouble(); // sec
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_VALVEPOSITION_SINEWAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ACTPOSITION_MOVESMOOTH_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_ACTPOSITION_MOVESMOOTH_POS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_ACTPOSITION_MOVESMOOTH_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_ACTPOSITION_MOVESMOOTH;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ACTPOSITION_SINEWAVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_ACTPOSITION_SINEWAVE_NUM->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_ACTPOSITION_SINEWAVE_MAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_ACTPOSITION_SINEWAVE_PER->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_ACTPOSITION_SINEWAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ACTPOSITION_SINEWAVE_MULTI_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_ACTPOSITION_SINEWAVE_MULTI;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ACTFORCE_STEPINPUT_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_ACTFORCE_STEPINPUT_VALUE->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_ACTFORCE_STEPINPUT_STIFFNESS->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_ACTFORCE_STEPINPUT_DAMPING->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_ACTFORCE_STEP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ACTFORCE_GRAVCOMP_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = false; // Grav. Comp. Go!
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_ACTFORCE_GRAVCOMP_MASS->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_ACTFORCE_GRAVCOMP_STIFFNESS->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_ACTFORCE_GRAVCOMP_DAMPING->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_ACTFORCE_GRAVCOMP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ACTFORCE_GRAVCOMP_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = true; // Grav. Comp. Stop!
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_ACTFORCE_GRAVCOMP;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void ValvePerfTest_Dialog::on_BTN_ACTFORCE_SINEWAVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO->text().toInt(); //BNO
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_ACTFORCE_SINEWAVE_MASS->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_ACTFORCE_SINEWAVE_STIFFNESS->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_ACTFORCE_SINEWAVE_DAMPING->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_ACTFORCE_SINEWAVE_NUM->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_ACTFORCE_SINEWAVE_MAG->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_ACTFORCE_SINEWAVE_PER->text().toDouble(); // N or Nm
    cmd.COMMAND_DATA.USER_COMMAND = VALVEPERFTEST_ACTFORCE_SINEWAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}



