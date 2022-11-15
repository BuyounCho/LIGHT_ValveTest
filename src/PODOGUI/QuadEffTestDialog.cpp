#include "QuadEffTestDialog.h"
#include "ui_QuadEffTestDialog.h"

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>
using namespace std;


QuadEffTestDialog::QuadEffTestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QuadEffTestDialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("QuadEffTest");
    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateDatas()));
}

QuadEffTestDialog::~QuadEffTestDialog()
{
    delete ui;
}

inline void DisplayMechaPower(QLineEdit *edit){
    edit->setText(QString().sprintf("%.1f", PODO_DATA.CoreSEN.PUMP[0].CurrentMechaPower));
}

inline void DisplayHydroPower(QLineEdit *edit){
    edit->setText(QString().sprintf("%.1f", PODO_DATA.CoreSEN.PUMP[0].CurrentHydraPower));
}

void QuadEffTestDialog::UpdateDatas(){
    DisplayMechaPower(ui->DISP_MECHAPOWER);
    DisplayHydroPower(ui->DISP_HYDROPOWER);
}

void QuadEffTestDialog::on_BTN_HOMEPOSE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_HOMEPOSE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_SQUAT_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_SQUAT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->EDIT_SQUAT_REPS->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_SQUAT_PERIOD->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_SQUAT_Z_POS->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_SQUAT_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_SQUAT;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_SWING_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_SWING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->EDIT_SWING_REPS->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_SWING_PERIOD->text().toDouble();;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_SWING_LENGTH->text().toDouble();;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->EDIT_SWING_HEIGHT->text().toDouble();;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_SWING_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_SWING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

int QuadEffTestDialog::Get_TESTJOINT_JointSelected()
{
    if(ui->RB_TESTJOINT_HIP_JOINT->isChecked()){return 0;}
    if(ui->RB_TESTJOINT_KNEE_JOINT->isChecked()){return 1;}
    return -1;
}

void QuadEffTestDialog::on_BTN_TESTJOINT_MOVEJOINT_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_TESTJOINTS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0; // MoveJoint
    cmd.COMMAND_DATA.USER_PARA_INT[2] = Get_TESTJOINT_JointSelected();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_TESTJOINT_MOVEJOINT_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_TESTJOINT_MOVEJOINT_ANGLE->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_TESTJOINT_SINEWAVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_TESTJOINTS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1; // SineWave
    cmd.COMMAND_DATA.USER_PARA_INT[2] = Get_TESTJOINT_JointSelected();
    cmd.COMMAND_DATA.USER_PARA_INT[3] = ui->EDIT_TESTJOINT_SINEWAVE_REPS->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_TESTJOINT_SINEWAVE_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_TESTJOINT_SINEWAVE_MAG->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_ARMTASK_MOVEJOINT_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_ARMMOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0; // MoveJoint
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_ARMTASK_MOVEJOINT_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_ARMTASK_MOVEJOINT_ANGLE->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->EDIT_ARMTASK_LOADMASS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->EDIT_ARMTASK_ARMLENGTH->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_ARMTASK_SINEWAVE_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_ARMMOTION;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1; // MoveJoint
    cmd.COMMAND_DATA.USER_PARA_INT[2] = ui->EDIT_ARMTASK_SINEWAVE_REPS->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_ARMTASK_SINEWAVE_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_ARMTASK_SINEWAVE_MAG->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->EDIT_ARMTASK_LOADMASS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->EDIT_ARMTASK_ARMLENGTH->text().toDouble();
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = true;
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void QuadEffTestDialog::on_BTN_SAVE_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = false; // save flag off
    string filename = ui->EDIT_SAVE_FILENAME->text().toStdString(); // filename string
    if(filename.empty()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = NULL;
    } else {
        strcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, filename.c_str());
    }
    cmd.COMMAND_DATA.USER_COMMAND = QUAD_TASK_SAVE_DATA;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}
