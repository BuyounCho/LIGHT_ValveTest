#include "JointDialog.h"
#include "ui_JointDialog.h"

// Display Reference
inline void DisplayJointReference(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentRefAngle));
}
inline void DisplayJointReferenceVelocity(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentRefAngVel));
}
inline void DisplayJointReferenceTorque(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentRefTorque));
}
inline void DisplayActuatorReference(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentRefActPos));
}
inline void DisplayActuatorReferenceVelocity(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentRefActVel));
}

// Display Position Data
inline void DisplayJointEncoder(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentAngle));
}

inline void DisplayActuatorEncoder(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentActPos));
}

// Display SensorData and BoardData
inline void DisplayJointPressureA(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentPressureA));
}
inline void DisplayJointPressureB(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentPressureB));
}
inline void DisplayJointLoadCell(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentActForce));
}
inline void DisplayJointValvePos(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentValvePos));
//    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentRefValve));
}
inline void DisplayJointTorque(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentTorque));
}

// Display Joint Control Mode
inline void DisplayJointControlMode(int jnum, QLineEdit *edit){

    QPalette *palette = new QPalette();
    switch(PODO_DATA.CoreREF.ValveCtrlMode[jnum]) {
    case ValveControlMode_Null: // Valve Control Mode : Null
    {
        edit->setText(QString().sprintf("%s", "NULL"));
        QPalette *palette = new QPalette();
        palette->setColor(QPalette::Text,Qt::lightGray);
        edit->setPalette(*palette);
        break;
    }
    case ValveControlMode_PosOrFor: // Valve Control Mode : Position or Force Control
    {
        if(PODO_DATA.CoreREF.PosOrFor_Selection[jnum] == JointControlMode_Torque) { // torque control mode
            edit->setText(QString().sprintf("%s", "TOR"));
            QPalette *palette = new QPalette();
            palette->setColor(QPalette::Text,Qt::red);
            edit->setPalette(*palette);
        } else if(PODO_DATA.CoreREF.PosOrFor_Selection[jnum] == JointControlMode_Position) { // position control mode
            edit->setText(QString().sprintf("%s", "POS"));
            QPalette *palette = new QPalette();
            palette->setColor(QPalette::Text,Qt::blue);
            edit->setPalette(*palette);
        }
        break;
    }
    case ValveControlMode_Opening: // Valve Control Mode : Valve Opening
    {
        edit->setText(QString().sprintf("%s", "OPEN"));
        palette->setColor(QPalette::Text,Qt::darkGreen);
        edit->setPalette(*palette);
        break;
    }
    case ValveControlMode_PWM: // Valve Control Mode : Valve input voltage
    {
        edit->setText(QString().sprintf("%s", "PWM"));
        palette->setColor(QPalette::Text,Qt::cyan);
        edit->setPalette(*palette);
        break;
    }
    case ValveControlMode_UtilMode: // Valve Control Mode : Utility mode (such as homepose, parameter ID..)
    {
        edit->setText(QString().sprintf("%s", "UTIL"));
        palette->setColor(QPalette::Text,Qt::magenta);
        edit->setPalette(*palette);
        break;
    }
    default:
        break;
    }
}

// Display Pump Control Mode
inline void DisplayPumpControlMode(int jnum, QLineEdit *edit){

    QPalette *palette = new QPalette();
    switch(PODO_DATA.CoreREF.PumpCtrlMode[jnum]) {
    case PumpControlMode_Null:
    {
        edit->setText(QString().sprintf("%s", "NULL"));
        QPalette *palette = new QPalette();
        palette->setColor(QPalette::Text,Qt::lightGray);
        edit->setPalette(*palette);
        break;
    }
    case PumpControlMode_Interpolation:
    {
        edit->setText(QString().sprintf("%s", "INT"));
        QPalette *palette = new QPalette();
        palette->setColor(QPalette::Text,Qt::blue);
        edit->setPalette(*palette);
        break;
    }
    case PumpControlMode_ActiveControl:
    {
        edit->setText(QString().sprintf("%s", "ACT"));
        palette->setColor(QPalette::Text,Qt::red);
        edit->setPalette(*palette);
        break;
    }
    default:
        break;
    }
}

JointDialog::JointDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JointDialog)
{
    ui->setupUi(this);

    // Data Table Update
    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateJoints()));
//    connect(pLAN->readTimer, SIGNAL(timeout()), this, SLOT(UpdateJoints()));


//    // Graph Plot Init.
//    Freq_GraphUpdate = 30;
//    GraphInitialize();
//    readTimer = new QTimer(this);
//    connect(readTimer, SIGNAL(timeout()), this, SLOT(GraphUpdate_RightLeg()));
////    connect(readTimer, SIGNAL(timeout()), this, SLOT(GraphUpdate_Pump()));
//    readTimer->start(Freq_GraphUpdate);

////    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(GraphUpdate_RightLeg()));
////    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(GraphUpdate_IMU()));
////    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(GraphUpdate_Pump()));
}

JointDialog::~JointDialog()
{
    delete ui;
}

void JointDialog::UpdateJoints(){

    DisplayJointReference(RHR  , ui->LE_JOINT_RHR_REFPOS);
    DisplayJointReference(RHY  , ui->LE_JOINT_RHY_REFPOS);
    DisplayJointReference(RHP  , ui->LE_JOINT_RHP_REFPOS);
    DisplayJointReference(RKN  , ui->LE_JOINT_RKN_REFPOS);
    DisplayJointReference(RAP  , ui->LE_JOINT_RAP_REFPOS);
    DisplayJointReference(RAR  , ui->LE_JOINT_RAR_REFPOS);
    DisplayJointReference(LHR  , ui->LE_JOINT_LHR_REFPOS);
    DisplayJointReference(LHY  , ui->LE_JOINT_LHY_REFPOS);
    DisplayJointReference(LHP  , ui->LE_JOINT_LHP_REFPOS);
    DisplayJointReference(LKN  , ui->LE_JOINT_LKN_REFPOS);
    DisplayJointReference(LAP  , ui->LE_JOINT_LAP_REFPOS);
    DisplayJointReference(LAR  , ui->LE_JOINT_LAR_REFPOS);
    DisplayJointReference(WST  , ui->LE_JOINT_WST_REFPOS);

    DisplayJointEncoder(RHR  , ui->LE_JOINT_RHR_POS);
    DisplayJointEncoder(RHY  , ui->LE_JOINT_RHY_POS);
    DisplayJointEncoder(RHP  , ui->LE_JOINT_RHP_POS);
    DisplayJointEncoder(RKN  , ui->LE_JOINT_RKN_POS);
    DisplayJointEncoder(RAP  , ui->LE_JOINT_RAP_POS);
    DisplayJointEncoder(RAR  , ui->LE_JOINT_RAR_POS);
    DisplayJointEncoder(LHR  , ui->LE_JOINT_LHR_POS);
    DisplayJointEncoder(LHY  , ui->LE_JOINT_LHY_POS);
    DisplayJointEncoder(LHP  , ui->LE_JOINT_LHP_POS);
    DisplayJointEncoder(LKN  , ui->LE_JOINT_LKN_POS);
    DisplayJointEncoder(LAP  , ui->LE_JOINT_LAP_POS);
    DisplayJointEncoder(LAR  , ui->LE_JOINT_LAR_POS);
    DisplayJointEncoder(WST  , ui->LE_JOINT_WST_POS);

    DisplayActuatorEncoder(RHR  , ui->LE_JOINT_RHR_LIN);
    DisplayActuatorEncoder(RHY  , ui->LE_JOINT_RHY_LIN);
    DisplayActuatorEncoder(RHP  , ui->LE_JOINT_RHP_LIN);
    DisplayActuatorEncoder(RKN  , ui->LE_JOINT_RKN_LIN);
    DisplayActuatorEncoder(RAP  , ui->LE_JOINT_RAP_LIN);
    DisplayActuatorEncoder(RAR  , ui->LE_JOINT_RAR_LIN);
    DisplayActuatorEncoder(LHR  , ui->LE_JOINT_LHR_LIN);
    DisplayActuatorEncoder(LHY  , ui->LE_JOINT_LHY_LIN);
    DisplayActuatorEncoder(LHP  , ui->LE_JOINT_LHP_LIN);
    DisplayActuatorEncoder(LKN  , ui->LE_JOINT_LKN_LIN);
    DisplayActuatorEncoder(LAP  , ui->LE_JOINT_LAP_LIN);
    DisplayActuatorEncoder(LAR  , ui->LE_JOINT_LAR_LIN);
    DisplayActuatorEncoder(WST  , ui->LE_JOINT_WST_LIN);
//    DisplayActuatorReference(RHR  , ui->LE_JOINT_RHR_LIN);
//    DisplayActuatorReference(RHY  , ui->LE_JOINT_RHY_LIN);
//    DisplayActuatorReference(RHP  , ui->LE_JOINT_RHP_LIN);
//    DisplayActuatorReference(RKN  , ui->LE_JOINT_RKN_LIN);
//    DisplayActuatorReference(RAP  , ui->LE_JOINT_RAP_LIN);
//    DisplayActuatorReference(RAR  , ui->LE_JOINT_RAR_LIN);
//    DisplayActuatorReference(LHR  , ui->LE_JOINT_LHR_LIN);
//    DisplayActuatorReference(LHY  , ui->LE_JOINT_LHY_LIN);
//    DisplayActuatorReference(LHP  , ui->LE_JOINT_LHP_LIN);
//    DisplayActuatorReference(LKN  , ui->LE_JOINT_LKN_LIN);
//    DisplayActuatorReference(LAP  , ui->LE_JOINT_LAP_LIN);
//    DisplayActuatorReference(LAR  , ui->LE_JOINT_LAR_LIN);
//    DisplayActuatorReference(WST  , ui->LE_JOINT_WST_LIN);

    DisplayJointTorque(RHR  , ui->LE_JOINT_RHR_TORQUE);
    DisplayJointTorque(RHY  , ui->LE_JOINT_RHY_TORQUE);
    DisplayJointTorque(RHP  , ui->LE_JOINT_RHP_TORQUE);
    DisplayJointTorque(RKN  , ui->LE_JOINT_RKN_TORQUE);
    DisplayJointTorque(RAP  , ui->LE_JOINT_RAP_TORQUE);
    DisplayJointTorque(RAR  , ui->LE_JOINT_RAR_TORQUE);
    DisplayJointTorque(LHR  , ui->LE_JOINT_LHR_TORQUE);
    DisplayJointTorque(LHY  , ui->LE_JOINT_LHY_TORQUE);
    DisplayJointTorque(LHP  , ui->LE_JOINT_LHP_TORQUE);
    DisplayJointTorque(LKN  , ui->LE_JOINT_LKN_TORQUE);
    DisplayJointTorque(LAP  , ui->LE_JOINT_LAP_TORQUE);
    DisplayJointTorque(LAR  , ui->LE_JOINT_LAR_TORQUE);
    DisplayJointTorque(WST  , ui->LE_JOINT_WST_TORQUE);

    DisplayJointLoadCell(RHR  , ui->LE_JOINT_RHR_LC);
    DisplayJointLoadCell(RHY  , ui->LE_JOINT_RHY_LC);
    DisplayJointLoadCell(RHP  , ui->LE_JOINT_RHP_LC);
    DisplayJointLoadCell(RKN  , ui->LE_JOINT_RKN_LC);
    DisplayJointLoadCell(RAP  , ui->LE_JOINT_RAP_LC);
    DisplayJointLoadCell(RAR  , ui->LE_JOINT_RAR_LC);
    DisplayJointLoadCell(LHR  , ui->LE_JOINT_LHR_LC);
    DisplayJointLoadCell(LHY  , ui->LE_JOINT_LHY_LC);
    DisplayJointLoadCell(LHP  , ui->LE_JOINT_LHP_LC);
    DisplayJointLoadCell(LKN  , ui->LE_JOINT_LKN_LC);
    DisplayJointLoadCell(LAP  , ui->LE_JOINT_LAP_LC);
    DisplayJointLoadCell(LAR  , ui->LE_JOINT_LAR_LC);
    DisplayJointLoadCell(WST  , ui->LE_JOINT_WST_LC);

    DisplayJointReferenceTorque(RHR  , ui->LE_JOINT_RHR_REFFORCE);
    DisplayJointReferenceTorque(RHY  , ui->LE_JOINT_RHY_REFFORCE);
    DisplayJointReferenceTorque(RHP  , ui->LE_JOINT_RHP_REFFORCE);
    DisplayJointReferenceTorque(RKN  , ui->LE_JOINT_RKN_REFFORCE);
    DisplayJointReferenceTorque(RAP  , ui->LE_JOINT_RAP_REFFORCE);
    DisplayJointReferenceTorque(RAR  , ui->LE_JOINT_RAR_REFFORCE);
    DisplayJointReferenceTorque(LHR  , ui->LE_JOINT_LHR_REFFORCE);
    DisplayJointReferenceTorque(LHY  , ui->LE_JOINT_LHY_REFFORCE);
    DisplayJointReferenceTorque(LHP  , ui->LE_JOINT_LHP_REFFORCE);
    DisplayJointReferenceTorque(LKN  , ui->LE_JOINT_LKN_REFFORCE);
    DisplayJointReferenceTorque(LAP  , ui->LE_JOINT_LAP_REFFORCE);
    DisplayJointReferenceTorque(LAR  , ui->LE_JOINT_LAR_REFFORCE);
    DisplayJointReferenceTorque(WST  , ui->LE_JOINT_WST_REFFORCE);

    DisplayJointValvePos(RHR  , ui->LE_JOINT_RHR_VP);
    DisplayJointValvePos(RHY  , ui->LE_JOINT_RHY_VP);
    DisplayJointValvePos(RHP  , ui->LE_JOINT_RHP_VP);
    DisplayJointValvePos(RKN  , ui->LE_JOINT_RKN_VP);
    DisplayJointValvePos(RAP  , ui->LE_JOINT_RAP_VP);
    DisplayJointValvePos(RAR  , ui->LE_JOINT_RAR_VP);
    DisplayJointValvePos(LHR  , ui->LE_JOINT_LHR_VP);
    DisplayJointValvePos(LHY  , ui->LE_JOINT_LHY_VP);
    DisplayJointValvePos(LHP  , ui->LE_JOINT_LHP_VP);
    DisplayJointValvePos(LKN  , ui->LE_JOINT_LKN_VP);
    DisplayJointValvePos(LAP  , ui->LE_JOINT_LAP_VP);
    DisplayJointValvePos(LAR  , ui->LE_JOINT_LAR_VP);
    DisplayJointValvePos(WST  , ui->LE_JOINT_WST_VP);

    DisplayJointPressureA(RHR  , ui->LE_JOINT_RHR_PA);
    DisplayJointPressureA(RHY  , ui->LE_JOINT_RHY_PA);
    DisplayJointPressureA(RHP  , ui->LE_JOINT_RHP_PA);
    DisplayJointPressureA(RKN  , ui->LE_JOINT_RKN_PA);
    DisplayJointPressureA(RAP  , ui->LE_JOINT_RAP_PA);
    DisplayJointPressureA(RAR  , ui->LE_JOINT_RAR_PA);
    DisplayJointPressureA(LHR  , ui->LE_JOINT_LHR_PA);
    DisplayJointPressureA(LHY  , ui->LE_JOINT_LHY_PA);
    DisplayJointPressureA(LHP  , ui->LE_JOINT_LHP_PA);
    DisplayJointPressureA(LKN  , ui->LE_JOINT_LKN_PA);
    DisplayJointPressureA(LAP  , ui->LE_JOINT_LAP_PA);
    DisplayJointPressureA(LAR  , ui->LE_JOINT_LAR_PA);
    DisplayJointPressureA(WST  , ui->LE_JOINT_WST_PA);

    DisplayJointPressureB(RHR  , ui->LE_JOINT_RHR_PB);
    DisplayJointPressureB(RHY  , ui->LE_JOINT_RHY_PB);
    DisplayJointPressureB(RHP  , ui->LE_JOINT_RHP_PB);
    DisplayJointPressureB(RKN  , ui->LE_JOINT_RKN_PB);
    DisplayJointPressureB(RAP  , ui->LE_JOINT_RAP_PB);
    DisplayJointPressureB(RAR  , ui->LE_JOINT_RAR_PB);
    DisplayJointPressureB(LHR  , ui->LE_JOINT_LHR_PB);
    DisplayJointPressureB(LHY  , ui->LE_JOINT_LHY_PB);
    DisplayJointPressureB(LHP  , ui->LE_JOINT_LHP_PB);
    DisplayJointPressureB(LKN  , ui->LE_JOINT_LKN_PB);
    DisplayJointPressureB(LAP  , ui->LE_JOINT_LAP_PB);
    DisplayJointPressureB(LAR  , ui->LE_JOINT_LAR_PB);
    DisplayJointPressureB(WST  , ui->LE_JOINT_WST_PB);

    DisplayJointControlMode(RHR, ui->LE_JOINT_RHR_CTRL_MODE);
    DisplayJointControlMode(RHY, ui->LE_JOINT_RHY_CTRL_MODE);
    DisplayJointControlMode(RHP, ui->LE_JOINT_RHP_CTRL_MODE);
    DisplayJointControlMode(RKN, ui->LE_JOINT_RKN_CTRL_MODE);
    DisplayJointControlMode(RAP, ui->LE_JOINT_RAK1_CTRL_MODE);
    DisplayJointControlMode(RAR, ui->LE_JOINT_RAK2_CTRL_MODE);
    DisplayJointControlMode(LHR, ui->LE_JOINT_LHR_CTRL_MODE);
    DisplayJointControlMode(LHY, ui->LE_JOINT_LHY_CTRL_MODE);
    DisplayJointControlMode(LHP, ui->LE_JOINT_LHP_CTRL_MODE);
    DisplayJointControlMode(LKN, ui->LE_JOINT_LKN_CTRL_MODE);
    DisplayJointControlMode(LAP, ui->LE_JOINT_LAK1_CTRL_MODE);
    DisplayJointControlMode(LAR, ui->LE_JOINT_LAK2_CTRL_MODE);
    DisplayJointControlMode(WST, ui->LE_JOINT_WST_CTRL_MODE);

    DisplayPumpControlMode(0, ui->LE_PUMP_CTRL_MODE);

}

QCustomPlot* PlotSet_R[6];
QCustomPlot* PlotSet_L[6];
QCustomPlot* PlotSet_I[6];
QCustomPlot* PlotSet_P[2];
void JointDialog::GraphInitialize()
{
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    //timeTicker->setTimeFormat("%h:%m:%s");
    timeTicker->setTimeFormat("%s");

    QPen myPen, dotPen, dotPen2;

    // =================================================================================================

    PlotSet_R[0] = ui->plot_RHR;
    PlotSet_R[1] = ui->plot_RHY;
    PlotSet_R[2] = ui->plot_RHP;
    PlotSet_R[3] = ui->plot_RKN;
    PlotSet_R[4] = ui->plot_RAP;
    PlotSet_R[5] = ui->plot_RAR;

    for(int i=0; i<6; i++)
    {
//        PlotSet_R[i]->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
        PlotSet_R[i]->addGraph();
        PlotSet_R[i]->addGraph();
        PlotSet_R[i]->xAxis->setTicker(timeTicker);
        PlotSet_R[i]->axisRect()->setupFullAxesBox();
        PlotSet_R[i]->xAxis->setRange(0.0, 5.0);
        PlotSet_R[i]->yAxis->setRange(-20.0, 20.0);

        dotPen.setStyle(Qt::DotLine);
        dotPen.setColor(Qt::gray);
        dotPen.setWidth(20);
        dotPen.setWidthF(2);
        PlotSet_R[i]->graph(0)->setPen(dotPen);
        myPen.setColor(Qt::red);
        myPen.setWidthF(1);
        PlotSet_R[i]->graph(1)->setPen(myPen);
    }

    // =================================================================================================

    PlotSet_I[0] = ui->plot_RollAngle;
    PlotSet_I[1] = ui->plot_PitchAngle;
    PlotSet_I[2] = ui->plot_YawAngle;
    PlotSet_I[3] = ui->plot_RollVel;
    PlotSet_I[4] = ui->plot_PitchVel;
    PlotSet_I[5] = ui->plot_YawVel;

    for(int i=0; i<6; i++)
    {
//        PlotSet_I[i]->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
        PlotSet_I[i]->addGraph();
        PlotSet_I[i]->xAxis->setTicker(timeTicker);
        PlotSet_I[i]->axisRect()->setupFullAxesBox();
        PlotSet_I[i]->xAxis->setRange(0.0, 3.0);
        PlotSet_I[i]->yAxis->setRange(-30.0, 30.0);

        if(i==0) myPen.setColor(Qt::blue);
        if(i==1) myPen.setColor(Qt::green);
        if(i==2) myPen.setColor(Qt::red);
        if(i==3) myPen.setColor(Qt::cyan);
        if(i==4) myPen.setColor(Qt::darkYellow);
        if(i==5) myPen.setColor(Qt::magenta);
        myPen.setWidthF(2.5);
        PlotSet_I[i]->graph(0)->setPen(myPen);
    }

    // =================================================================================================

    temp_time.resize(150);
    temp_time_prev.resize(MAX_PREVIEW+1);
    temp_PumpPressureRef.resize(150);
    temp_PumpPressureRef_prev.resize(MAX_PREVIEW+1);
    temp_PumpPressure.resize(150);

    PlotSet_P[0] = ui->plot_PumpPressure;
    PlotSet_P[0]->plotLayout()->insertRow(0);
    PlotSet_P[0]->plotLayout()->addElement(0, 0, new QCPTextElement(PlotSet_P[0], "Pump Pressure", QFont("Arial", 16, QFont::Bold)));
    PlotSet_P[0]->addGraph();
    PlotSet_P[0]->addGraph();
    PlotSet_P[0]->addGraph();
    PlotSet_P[0]->xAxis->setTicker(timeTicker);
    PlotSet_P[0]->axisRect()->setupFullAxesBox();
    double T = PODO_DATA.CoreREF.dT_PrevPump*(double)PODO_DATA.CoreREF.N_PrevPump;
    PlotSet_P[0]->xAxis->setRange(-T,T);
    PlotSet_P[0]->yAxis->setRange(-10.0, 150.0);

    for(int i=0;i<150;i++) {
        temp_time[i] = 0.001*(double)(Freq_GraphUpdate*(i-149));
    }
    for(int i=0;i<(MAX_PREVIEW+1);i++) {
        temp_time_prev[i] = PODO_DATA.CoreREF.dT_PrevPump*(double)i;
    }
    PlotSet_P[0]->graph(0)->addData(temp_time,temp_PumpPressureRef);
    PlotSet_P[0]->graph(1)->addData(temp_time,temp_PumpPressure);
    PlotSet_P[0]->graph(2)->addData(temp_time_prev,temp_PumpPressureRef_prev);


    QSharedPointer<QCPAxisTickerFixed> fixedTicker(new QCPAxisTickerFixed);
    PlotSet_P[0]->xAxis->setTicker(fixedTicker);
    fixedTicker->setTickStep(0.5); // tick step shall be 1.0
    fixedTicker->setScaleStrategy(QCPAxisTickerFixed::ssNone); // and no scaling of the tickstep (like multiples or powers) is allowed

    dotPen.setStyle(Qt::DotLine);
    dotPen.setColor(Qt::gray);
    dotPen.setWidth(20);
    dotPen.setWidthF(2);
    PlotSet_P[0]->graph(0)->setPen(dotPen);
    myPen.setColor(Qt::red);
    myPen.setWidthF(1);
    PlotSet_P[0]->graph(1)->setPen(myPen);

    QCPItemStraightLine *infLine = new QCPItemStraightLine(PlotSet_P[0]);
    infLine->point1->setCoords(0, 0);  // location of point 1 in plot coordinate
    infLine->point2->setCoords(0, 1);  // location of point 2 in plot coordinate
    dotPen2.setStyle(Qt::DotLine);
    dotPen2.setColor(Qt::black);
    dotPen2.setWidth(30);
    dotPen2.setWidthF(3);
    infLine->setPen(dotPen2);

    QCPItemRect* section = new QCPItemRect(PlotSet_P[0]);
    section->topLeft->setType(QCPItemPosition::ptPlotCoords);
    section->topLeft->setAxes(PlotSet_P[0]->xAxis, PlotSet_P[0]->yAxis);
    section->bottomRight->setType(QCPItemPosition::ptPlotCoords);
    section->bottomRight->setAxes(PlotSet_P[0]->xAxis, PlotSet_P[0]->yAxis);
    section->topLeft->setCoords(0.0,150.0);
    section->bottomRight->setCoords(2.5,0.0);
    section->setBrush(QBrush(QColor(0,0,200,20))); // RGB and transparency
    section->setPen(Qt::NoPen);
    PlotSet_P[0]->addLayer("sectionBackground", PlotSet_P[0]->layer("grid"), QCustomPlot::limBelow);
    section->setLayer("sectionBackground");

    QCPItemText *textLabel = new QCPItemText(PlotSet_P[0]);
    textLabel->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
    textLabel->position->setType(QCPItemPosition::ptAxisRectRatio);
    textLabel->position->setCoords(0.75, 0.05); // place position at center/top of axis rect
    textLabel->setText("Previewed Reference");
    textLabel->setFont(QFont("Arial", 12)); // make font a bit larger
    textLabel->setPen(Qt::NoPen);

    QCPItemText *textLabel2 = new QCPItemText(PlotSet_P[0]);
    textLabel2->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
    textLabel2->position->setType(QCPItemPosition::ptAxisRectRatio);
    textLabel2->position->setCoords(0.25, 0.05); // place position at center/top of axis rect
    textLabel2->setText("Pressure Tracking");
    textLabel2->setFont(QFont("Arial", 12)); // make font a bit larger
    textLabel2->setPen(Qt::NoPen);

    // =================================================================================================

    temp_PumpFlowrateRef.resize(150);
    temp_PumpFlowrate.resize(150);

    PlotSet_P[1] = ui->plot_PumpFlowrate;
    PlotSet_P[1]->plotLayout()->insertRow(0);
    PlotSet_P[1]->plotLayout()->addElement(0, 0, new QCPTextElement(PlotSet_P[1], "Pump Flowrate", QFont("Arial", 16, QFont::Bold)));
    PlotSet_P[1]->addGraph();
    PlotSet_P[1]->addGraph();
    PlotSet_P[1]->xAxis->setTicker(timeTicker);
    PlotSet_P[1]->axisRect()->setupFullAxesBox();
    PlotSet_P[1]->xAxis->setRange(-T,T);
    PlotSet_P[1]->yAxis->setRange(-100.0, 3000.0);

    QSharedPointer<QCPAxisTickerFixed> fixedTicker2(new QCPAxisTickerFixed);
    PlotSet_P[1]->xAxis->setTicker(fixedTicker);
    fixedTicker2->setTickStep(0.5); // tick step shall be 1.0
    fixedTicker2->setScaleStrategy(QCPAxisTickerFixed::ssNone); // and no scaling of the tickstep (like multiples or powers) is allowed

    dotPen.setStyle(Qt::DotLine);
    dotPen.setColor(Qt::gray);
    dotPen.setWidth(20);
    dotPen.setWidthF(2);
    PlotSet_P[1]->graph(0)->setPen(dotPen);
    myPen.setColor(Qt::red);
    myPen.setWidthF(1);
    PlotSet_P[1]->graph(1)->setPen(myPen);

    QCPItemStraightLine *infLine2 = new QCPItemStraightLine(PlotSet_P[1]);
    infLine2->point1->setCoords(0, 0);  // location of point 1 in plot coordinate
    infLine2->point2->setCoords(0, 1);  // location of point 2 in plot coordinate
    dotPen2.setStyle(Qt::DotLine);
    dotPen2.setColor(Qt::black);
    dotPen2.setWidth(30);
    dotPen2.setWidthF(3);
    infLine2->setPen(dotPen2);

    QCPItemRect* section2 = new QCPItemRect(PlotSet_P[1]);
    section2->topLeft->setType(QCPItemPosition::ptPlotCoords);
    section2->topLeft->setAxes(PlotSet_P[1]->xAxis, PlotSet_P[1]->yAxis);
    section2->bottomRight->setType(QCPItemPosition::ptPlotCoords);
    section2->bottomRight->setAxes(PlotSet_P[1]->xAxis, PlotSet_P[1]->yAxis);
    section2->topLeft->setCoords(0.0,3000.0);
    section2->bottomRight->setCoords(2.5,0.0);
    section2->setBrush(QBrush(QColor(0,200,0,20))); // RGB and transparency
    section2->setPen(Qt::NoPen);
    PlotSet_P[1]->addLayer("sectionBackground", PlotSet_P[1]->layer("grid"), QCustomPlot::limBelow);
    section2->setLayer("sectionBackground");

    QCPItemText *textLabel3 = new QCPItemText(PlotSet_P[1]);
    textLabel3->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
    textLabel3->position->setType(QCPItemPosition::ptAxisRectRatio);
    textLabel3->position->setCoords(0.75, 0.05); // place position at center/top of axis rect
    textLabel3->setText("Previewed Reference");
    textLabel3->setFont(QFont("Arial", 12)); // make font a bit larger
    textLabel3->setPen(Qt::NoPen);

    QCPItemText *textLabel4 = new QCPItemText(PlotSet_P[1]);
    textLabel4->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
    textLabel4->position->setType(QCPItemPosition::ptAxisRectRatio);
    textLabel4->position->setCoords(0.25, 0.05); // place position at center/top of axis rect
    textLabel4->setText("Pressure Tracking");
    textLabel4->setFont(QFont("Arial", 12)); // make font a bit larger
    textLabel4->setPen(Qt::NoPen);
}

double find_max_element(double* arr, int max_idx) {
    double max_value = -1e+6;

    for(int idx = 0; idx<max_idx; idx++) {
        if(arr[idx] > max_value) {
            max_value = arr[idx];
        }
    }
    return max_value;
}

double find_min_element(double* arr, int max_idx) {
    double min_value = 1e+6;

    for(int idx = 0; idx<max_idx; idx++) {
        if(arr[idx] < min_value) {
            min_value = arr[idx];
        }
    }
    return min_value;
}

void JointDialog::GraphUpdate_RightLeg()
{
    static int dataidx = 0;
    static QTime time(QTime::currentTime());

    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    if (key-lastPointKey > 0.020) // at most add point every 20 ms
    {
        if(ui->radioButton_plot_ON->isChecked()) {

            for(int i=RHR; i<6; i++)
            {
                PlotSet_R[i]->graph(0)->data()->removeBefore(key-5.0);
                PlotSet_R[i]->graph(1)->data()->removeBefore(key-5.0);

                PlotSet_R[i]->graph(0)->addData(key, PODO_DATA.CoreSEN.ENCODER[i][0].CurrentRefAngle);
                PlotSet_R[i]->graph(1)->addData(key, PODO_DATA.CoreSEN.ENCODER[i][0].CurrentAngle);

                temp_angle[i][dataidx] = PODO_DATA.CoreSEN.ENCODER[i][0].CurrentRefAngle;
                temp_refangle[i][dataidx] = PODO_DATA.CoreSEN.ENCODER[i][0].CurrentAngle;
            }

            dataidx++;
            if(dataidx>300) {
                dataidx = 0;
            }


            for(int i=0; i<6; i++)
            {
                if(key>5.0) {
                    PlotSet_R[i]->xAxis->setRange(key-5.0, key);
                } else {
                    PlotSet_R[i]->xAxis->setRange(0.0, 5.0);
                }

                double max_refangle,min_refangle;
                max_refangle = find_max_element(temp_refangle[i],300);
                min_refangle = find_min_element(temp_refangle[i],300);
                double max_angle,min_angle;
                max_angle = find_max_element(temp_angle[i],300);
                min_angle = find_min_element(temp_angle[i],300);

                double max,min;
                if(max_refangle<max_angle) {
                    max = max_angle;
                } else {
                    max = max_refangle;
                }
                if(min_refangle<min_angle) {
                    min = min_refangle;
                } else {
                    min = min_angle;
                }

                PlotSet_R[i]->yAxis->setRange(min-20.0, max+20.0);
                PlotSet_R[i]->replot();
            }
        }

        lastPointKey = key;
    }
}

void JointDialog::GraphUpdate_IMU()
{
    static int dataidx = 0;
    static QTime time(QTime::currentTime());
    QString str;

    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;

    if (key-lastPointKey > 0.02) // at most add point every 20 ms
    {
        PlotSet_I[0]->graph(0)->addData(key, PODO_DATA.CoreSEN.IMU[0].Roll*R2D);
        PlotSet_I[1]->graph(0)->addData(key, PODO_DATA.CoreSEN.IMU[0].Pitch*R2D);
        PlotSet_I[2]->graph(0)->addData(key, PODO_DATA.CoreSEN.IMU[0].Yaw*R2D);
        PlotSet_I[3]->graph(0)->addData(key, PODO_DATA.CoreSEN.IMU[0].Wx_G*R2D);
        PlotSet_I[4]->graph(0)->addData(key, PODO_DATA.CoreSEN.IMU[0].Wy_G*R2D);
        PlotSet_I[5]->graph(0)->addData(key, PODO_DATA.CoreSEN.IMU[0].Wz_G*R2D);

        temp_IMU[0][dataidx]  = PODO_DATA.CoreSEN.IMU[0].Roll*R2D;
        temp_IMU[1][dataidx]  = PODO_DATA.CoreSEN.IMU[0].Pitch*R2D;
        temp_IMU[2][dataidx]  = PODO_DATA.CoreSEN.IMU[0].Yaw*R2D;
        temp_IMU[3][dataidx]  = PODO_DATA.CoreSEN.IMU[0].Wx_G*R2D;
        temp_IMU[4][dataidx]  = PODO_DATA.CoreSEN.IMU[0].Wy_G*R2D;
        temp_IMU[5][dataidx]  = PODO_DATA.CoreSEN.IMU[0].Wz_G*R2D;

        dataidx++;
        if(dataidx>150) {
            dataidx = 0;
        }

        if(ui->radioButton_plot_ON_3->isChecked()) {
            for(int i=0; i<6; i++)
            {
                if(key>3.0) {
                    PlotSet_I[i]->xAxis->setRange(key-3.0, key);
                } else {
                    PlotSet_I[i]->xAxis->setRange(0.0, 3.0);
                }

                double max_data,min_data;
                max_data = find_max_element(temp_IMU[i],300);
                min_data = find_min_element(temp_IMU[i],300);

                PlotSet_I[i]->yAxis->setRange(min_data-30.0, max_data+30.0);
                PlotSet_I[i]->replot();
            }
        }

        lastPointKey = key;
    }
}


void JointDialog::GraphUpdate_Pump()
{
    static int dataidx = 0;

    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
//    if (key-lastPointKey > 0.001*(double)Freq_GraphUpdate) // at most add point every 20 ms
    {
        // Plot Pressure ===============================================================================================
        if(dataidx<150-1) {
            for(int i=0;i<dataidx;i++) {
                temp_PumpPressureRef[i] = temp_PumpPressureRef[i+1];
                temp_PumpPressure[i] = temp_PumpPressure[i+1];
            }
            dataidx++;
        } else {
            for(int i=0;i<150-1;i++) {
                temp_PumpPressureRef[i] = temp_PumpPressureRef[i+1];
                temp_PumpPressure[i] = temp_PumpPressure[i+1];
            }
        }
        temp_PumpPressureRef[150-1] = PODO_DATA.CoreREF.PumpPressureReference[0];
        temp_PumpPressure[150-1] = PODO_DATA.CoreSEN.PUMP[0].CurrentPressure;

        temp_PumpPressureRef_prev[0] = PODO_DATA.CoreREF.RequiredPressureReference_Future[0];
        for(int i=0;i<MAX_PREVIEW;i++) {
            temp_PumpPressureRef_prev[i+1] = PODO_DATA.CoreREF.RequiredPressureReference_Future[i+1];
        }

        PlotSet_P[0]->graph(0)->setData(temp_time,temp_PumpPressureRef,true);
        PlotSet_P[0]->graph(1)->setData(temp_time,temp_PumpPressure,true);
        PlotSet_P[0]->graph(2)->setData(temp_time_prev,temp_PumpPressureRef_prev,true);
        
        QCPItemText *textLabel = new QCPItemText(PlotSet_P[0]);
        textLabel->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
        textLabel->position->setType(QCPItemPosition::ptAxisRectRatio);
        textLabel->position->setCoords(0.75, 0.25); // place position at center/top of axis rect
        int temp_n = PlotSet_P[0]->graph(0)->data()->size();
        textLabel->setText(QString().sprintf("%d", temp_n));
        textLabel->setFont(QFont("Arial", 12)); // make font a bit larger
        textLabel->setPen(Qt::NoPen);

        
        QCPItemTracer *Point = new QCPItemTracer(PlotSet_P[0]);
        Point->setGraph(PlotSet_P[0]->graph(0));
        Point->setGraphKey(0.0);
        Point->setInterpolating(true);
        Point->setStyle(QCPItemTracer::tsCircle);
        Point->setPen(QPen(Qt::red));
        Point->setBrush(Qt::red);
        Point->setSize(7);

        PlotSet_P[0]->replot();

        // Plot Flowrate ===============================================================================================
//        PlotSet_P[1]->graph(0)->data()->clear();
//        PlotSet_P[1]->graph(1)->data()->clear();

//        if(dataidx>=0) {
//            for(int i=149-dataidx;i>0;i--) {
//                temp_PumpFlowrateRef[i] = temp_PumpFlowrateRef[i-1];
//                temp_PumpFlowrate[i] = temp_PumpFlowrate[i-1];
//                PlotSet_P[1]->graph(0)->addData(temp_PumpTime[i] - key, temp_PumpFlowrateRef[i]);
//                PlotSet_P[1]->graph(1)->addData(temp_PumpTime[i] - key, temp_PumpFlowrate[i]);
//            }
//            dataidx--;
//        } else {
//            for(int i=149;i>0;i--) {
//                temp_PumpFlowrateRef[i] = temp_PumpFlowrateRef[i-1];
//                temp_PumpFlowrate[i] = temp_PumpFlowrate[i-1];
//                PlotSet_P[1]->graph(0)->addData(temp_PumpTime[i] - key, temp_PumpFlowrateRef[i]);
//                PlotSet_P[1]->graph(1)->addData(temp_PumpTime[i] - key, temp_PumpFlowrate[i]);
//            }
//        }
//        temp_PumpFlowrateRef[0] = PODO_DATA.CoreREF.PumpVelocityReference[0];
//        temp_PumpFlowrate[0] = PODO_DATA.CoreSEN.PUMP[0].CurrentVelocity;
//        PlotSet_P[1]->graph(0)->addData(0, temp_PumpFlowrateRef[0]);
//        PlotSet_P[1]->graph(1)->addData(0, temp_PumpFlowrate[0]);

//        for(int i=0;i<MAX_PREVIEW;i++) {
//            PlotSet_P[1]->graph(0)->addData((double)(i+1)*PODO_DATA.CoreREF.dT_window, PODO_DATA.CoreREF.RequiredFlowrateReference_Future[i+1]/9.39*3000.0);
//        }

//        QCPItemTracer *Point2 = new QCPItemTracer(PlotSet_P[1]);
//        Point2->setGraph(PlotSet_P[1]->graph(0));
//        Point2->setGraphKey(0.0);
//        Point2->setInterpolating(true);
//        Point2->setStyle(QCPItemTracer::tsCircle);
//        Point2->setPen(QPen(Qt::red));
//        Point2->setBrush(Qt::red);
//        Point2->setSize(7);

//        PlotSet_P[1]->replot();

        lastPointKey = key;
    }
}
