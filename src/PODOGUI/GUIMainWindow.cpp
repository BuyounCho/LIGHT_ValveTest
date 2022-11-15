#include "GUIMainWindow.h"
#include "ui_GUIMainWindow.h"

#include <iostream>



// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_AL           RBDataBase::_DB_AL[MAX_AL];

// LAN Data --------
LAN_PODO2GUI    PODO_DATA;
LANDialog       *pLAN;


using namespace std;

GUIMainWindow::GUIMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GUIMainWindow)
{
    ui->setupUi(this);
    this->move(0,0);

    RBDataBase DB;
    DB.SetFilename("Core_Config.db");
//    DB.SetFilename("Core_Config_ValveTest.db");
    if(DB.OpenDB() == false){
        FILE_LOG(logERROR) << "Fail to load database file";
    }

    dialogLAN = new LANDialog(this);
    pLAN = dialogLAN;
    dialogPODOAL = new PODOALDialog(this);

    dialogHCBSetting = new HCBSettingDialog(this);
    ui->MAIN_TAB->addTab((QWidget*)dialogHCBSetting, "HCB Setting");

//    dialogPumpControl = new PumpControlDialog(this);
//    ui->MAIN_TAB->addTab((QWidget*)dialogPumpControl, "Pump Control");

//    dialogLIGHTWalking = new LIGHTWalkingDialog(this);
//    ui->MAIN_TAB->addTab((QWidget*)dialogLIGHTWalking, "LIGHT Control 1");

//    dialogLIGHTWalking2 = new LIGHTWalkingDialog2(this);
//    ui->MAIN_TAB->addTab((QWidget*)dialogLIGHTWalking2, "LIGHT Control 2");

//    dialogLIGHTWalking_GainSetting = new LIGHTWalkingDialog_GainSetting(this);
//    ui->MAIN_TAB->addTab((QWidget*)dialogLIGHTWalking_GainSetting, "LIGHT Walk Gain");

//    dialogSH_TASK = new SH_TASK_Dialog(this);
//    ui->MAIN_TAB->addTab((QWidget*)dialogSH_TASK, "Joints Control");

    dialogValvePerfTest = new ValvePerfTest_Dialog(this);
    ui->MAIN_TAB->addTab((QWidget*)dialogValvePerfTest, "Valve Perf. Test");

    // Expandable Dialogs=============================
    expHandler  = new ExpandDialogHandler(this);
    frameJOINT  = new QFrame(this);
    frameSENSOR = new QFrame(this);

    dialogJOINT  = new JointDialog(frameJOINT);
    dialogSENSOR = new SensorDialog(frameSENSOR);

    expHandler->registerDialog(dialogJOINT, frameJOINT);
    expHandler->registerDialog(dialogSENSOR, frameSENSOR);

    // ===============================================

    icon_LAN_OFF.addFile(QStringLiteral("../share/GUI/icon/LAN_OFF_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
    icon_LAN_OFF.addFile(QStringLiteral("../share/GUI/icon/LAN_OFF_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
    icon_LAN_ON.addFile(QStringLiteral("../share/GUI/icon/LAN_ON_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
    icon_LAN_ON.addFile(QStringLiteral("../share/GUI/icon/LAN_ON_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
    icon_Module.addFile(QStringLiteral("../share/GUI/icon/MODULE_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
    icon_Module.addFile(QStringLiteral("../share/GUI/icon/MODULE_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
    icon_Joint.addFile(QStringLiteral("../share/GUI/icon/JOINT_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
    icon_Joint.addFile(QStringLiteral("../share/GUI/icon/JOINT_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
    icon_Sensor.addFile(QStringLiteral("../share/GUI/icon/SENSOR_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
    icon_Sensor.addFile(QStringLiteral("../share/GUI/icon/SENSOR_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
    ui->actionLAN->setIcon(icon_LAN_OFF);
    ui->actionPODOAL->setIcon(icon_Module);
    ui->actionJOINT->setIcon(icon_Joint);
    ui->actionSENSOR->setIcon(icon_Sensor);

    // Toolbar Signals================================
    connect(ui->actionLAN, SIGNAL(toggled(bool)), this, SLOT(ActionLAN_Toggled(bool)));
    connect(ui->actionPODOAL, SIGNAL(toggled(bool)), this, SLOT(ActionPODOAL_Toggled(bool)));

    connect(ui->actionJOINT,  SIGNAL(toggled(bool)), this, SLOT(ActionJOINT_Toggled()));
    connect(ui->actionSENSOR, SIGNAL(toggled(bool)), this, SLOT(ActionSENSOR_Toggled()));
    // ===============================================
    connect(dialogLAN, SIGNAL(SIG_LAN_ON_OFF(bool)), this, SLOT(SLOT_LAN_ONOFF(bool)));
    connect(dialogLAN, SIGNAL(SIG_LAN_HIDE()), this, SLOT(SLOT_LAN_HIDE()));
    connect(dialogPODOAL, SIGNAL(SIG_AL_HIDE()), this, SLOT(SLOT_AL_HIDE()));

    connect(dialogLAN, SIGNAL(NewPODOData()), this, SLOT(SLOT_NEW_DATA()));

}

GUIMainWindow::~GUIMainWindow()
{
    delete ui;
}

void GUIMainWindow::SLOT_NEW_DATA(){
    QString err;

    // Power ======================
    if(PODO_DATA.CoreSEN.SP[0].Voltage < 48.0){
        err += "Voltage Low ";
    }
    if(PODO_DATA.CoreSEN.SP[0].Current > 8.0){
        err += "Current High ";
    }

    ui->LB_VOLTAGE->setText(QString().sprintf("%.1fV", PODO_DATA.CoreSEN.SP[0].Voltage));
    ui->LB_CURRENT->setText(QString().sprintf("%.1fA", PODO_DATA.CoreSEN.SP[0].Current));
    // ============================

    // MC Stat ====================
    for(int i=0; i<NO_OF_JOINTS; i++){
        mSTAT ms = PODO_DATA.CoreSEN.MCStatus[MC_GetID(i)][MC_GetCH(i)];

        if((ms.B[1]&0x03) != 0){
            err += JointNameList[i] + " Power Over ";   // JAM PWM
        }
        if((ms.B[1]&0x0C) != 0){
            err += JointNameList[i] + " Input Wrong ";  // BIG INP
        }
        if((ms.B[1]&0x20) != 0){
            err += JointNameList[i] + " Enc Error ";    // ENC CUR(CAN)
        }
        if((ms.B[1]&0x40) != 0){
            err += JointNameList[i] + " CAN Error ";    // CUR(CAN)
        }
        if((ms.B[1]&0x80) != 0){
            err += JointNameList[i] + " Temp High ";    // TMP
        }
        if((ms.B[2]&0x03) != 0){
            err += JointNameList[i] + " Pos Limit ";    // PS1 PS2
        }
    }
    // ============================

    ui->statusBar->showMessage(err, 100);
}

void GUIMainWindow::ActionLAN_Toggled(bool checked){
    if(checked){
        dialogLAN->setWindowFlags(Qt::Popup);
        QPoint pt(10,10);
        pt = ui->centralWidget->mapToGlobal(pt);
        dialogLAN->move(pt);
        dialogLAN->show();
    }
}
void GUIMainWindow::ActionPODOAL_Toggled(bool checked){
    if(checked){
        dialogPODOAL->setWindowFlags(Qt::Popup);
        QPoint pt(10,100);
        pt = ui->centralWidget->mapToGlobal(pt);
        dialogPODOAL->move(pt);
        dialogPODOAL->show();
    }
}
void GUIMainWindow::SLOT_LAN_HIDE() {ui->actionLAN->setChecked(false);}
void GUIMainWindow::SLOT_AL_HIDE()  {ui->actionPODOAL->setChecked(false);}
void GUIMainWindow::SLOT_LAN_ONOFF(bool onoff){
    if(onoff){
        ui->actionLAN->setIcon(icon_LAN_ON);
    }else{
        ui->actionLAN->setIcon(icon_LAN_OFF);
    }
}


void GUIMainWindow::ActionJOINT_Toggled(){
    if(expHandler->isVisible(dialogJOINT))
        expHandler->hideDialog(dialogJOINT);
    else
        expHandler->showDialog(dialogJOINT);
}
void GUIMainWindow::ActionSENSOR_Toggled(){
    if(expHandler->isVisible(dialogSENSOR))
        expHandler->hideDialog(dialogSENSOR);
    else
        expHandler->showDialog(dialogSENSOR);
}

void GUIMainWindow::on_BTN_REF_ENABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;     // All joint
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;      // Position Ref
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = true;     // enable

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REF_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void GUIMainWindow::on_BTN_REF_DISABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;     // All joint
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;      // Position Ref
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = false;     // disable

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REF_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void GUIMainWindow::on_BTN_CAN_CHECK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_CAN_CHECK;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void GUIMainWindow::on_BTN_ENC_ENABLE_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_SENSOR_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
}

void GUIMainWindow::on_BTN_ENC_DISABLE_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // off
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_SENSOR_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
}
