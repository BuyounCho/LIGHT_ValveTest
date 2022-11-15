#include "hqdialog_threadmill.h"
#include "ui_hqdialog_threadmill.h"

#include "CommonHeader.h"

#include "BasicFiles/PODOALDialog.h"
#include "../ALPrograms/HuboQuad/Oinverse.h"
enum HUBOQUAD_ALCOMMAND
{
    HUBOQUAD_AL_NO_ACT = 100,
    HUBOQUAD_AL_READYPOS,
    HUBOQUAD_AL_WALK_START,
    HUBOQUAD_AL_WALK_STOP,
    HUBOQUAD_AL_LOCK,
    HUBOQUAD_AL_NOCON,
};



hqdialog_threadmill::hqdialog_threadmill(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::hqdialog_threadmill)
{
    ui->setupUi(this);
    AlnumHuboQuad = PODOALDialog::GetALNumFromFileName("HuboQuad");
    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    tt_y = 0;
    maxYstep = 0.04;//4cmperstep
    yadjust = false;
    displayTimer->start(50);

}

hqdialog_threadmill::~hqdialog_threadmill()
{
    delete ui;
}


void hqdialog_threadmill::DisplayUpdate()
{
    tt_y+=50.0/1000.0;
    if(yadjust)
    {
        if(tt_y>tleft_y)
        {
            on_BTN_Y0_clicked();
            yadjust = false;
        }
    }

    QString str;
    str.sprintf("RFFT %5d \nLFFT %5d\nRHFT %5d\nLHFT %5d\n"
                ,(int)PODO_DATA.UserM2G.RFFT,(int)PODO_DATA.UserM2G.LFFT
                ,(int)PODO_DATA.UserM2G.RHFT,(int)PODO_DATA.UserM2G.LHFT);
    ui->SENSORVIEW->setText(str);



    ui->LE_SENSOR_CIMU_ROLL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Roll));
    ui->LE_SENSOR_CIMU_PITCH->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Pitch));
    ui->LE_SENSOR_CIMU_ROLL_VEL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].RollVel));
    ui->LE_SENSOR_CIMU_PITCH_VEL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].PitchVel));
    ui->LE_SENSOR_CIMU_ROLL_ACC->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].AccX));
    ui->LE_SENSOR_CIMU_PITCH_ACC->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].AccY));

    ui->LE_SENSOR_CIMU_YAW->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Yaw));
    ui->LE_SENSOR_CIMU_YAW_VEL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].YawVel));

}

void hqdialog_threadmill::on_BTN_WALKSTART_clicked()
{

    if(ui->RB_TROT->isChecked())
    {
        DO_Trot();
    }
    else if(ui->RB_WAVETROT->isChecked())
    {
        ui->LE_DSP_RATIO->setText("0.0");
        ui->LE_OVERLAP->setText("0.5");
        ui->LE_STEP_T_3->setText("0.525");
        DO_Wave();
    }
    else if(ui->RB_WAVE->isChecked())
    {

        ui->LE_DSP_RATIO->setText("0.0");
        ui->LE_OVERLAP->setText("0.0");
        ui->LE_STEP_T_3->setText("0.7");
        DO_Wave();
    }
}



void hqdialog_threadmill::DO_Trot()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=ui->LE_STEP_LX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1]=ui->LE_STEP_LY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2]=ui->LE_STEP_LZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3]=ui->LE_STEP_ROT->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=ui->LE_STEP_T->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5]=ui->LE_STEP_FB_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6]=ui->LE_STEP_LR_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7]=ui->LE_COM_Z_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Trot;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}
void hqdialog_threadmill::DO_Wave()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=ui->LE_STEP_LX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1]=ui->LE_STEP_LY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2]=ui->LE_STEP_LZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3]=ui->LE_STEP_ROT->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=ui->LE_STEP_T_3->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5]=ui->LE_STEP_FB_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6]=ui->LE_STEP_LR_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7]=ui->LE_COM_Z_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8]=ui->LE_OVERLAP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9]=ui->LE_DSP_RATIO->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Wave2;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}

void  hqdialog_threadmill::DO_Standing()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=ui->LE_STEP_LX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1]=ui->LE_STEP_LY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2]=ui->LE_STEP_LZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3]=ui->LE_STEP_ROT->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=ui->LE_STEP_T_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5]=ui->LE_STEP_FB_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6]=ui->LE_STEP_LR_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7]=ui->LE_COM_Z_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Standing;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}

void hqdialog_threadmill::DO_adjustY(double y)
{
    double yoff = fabs(y);
    int pm = 1;
    if(y<0){ pm = -1;}
    double stepT = 1.0;
    if(ui->RB_TROT->isChecked()){stepT = ui->LE_STEP_T->text().toDouble();}
    if(ui->RB_WAVETROT->isChecked()){stepT = ui->LE_STEP_T_3->text().toDouble();}
    if(ui->RB_WAVE->isChecked()){stepT = ui->LE_STEP_T_3->text().toDouble();}

    double stepL = min(maxYstep,yoff);
    tleft_y = stepT*((int)(yoff/stepL)+1);
    stepL = yoff/((int)(yoff/stepL)+1);
    ui->LE_STEP_LY->setText(QString().sprintf("%.2f",pm*stepL));
    on_BTN_WALKSTART_clicked();
    tt_y = 0;
    yadjust = true;
}
void hqdialog_threadmill::on_BTN_L08_clicked()
{
    DO_adjustY(0.08);
}

void hqdialog_threadmill::on_BTN_L15_clicked()
{
    DO_adjustY(0.15);
}
void hqdialog_threadmill::on_BTN_L30_clicked()
{
    DO_adjustY(0.3);
}


void hqdialog_threadmill::on_BTN_R08_clicked()
{
    DO_adjustY(-0.08);
}
void hqdialog_threadmill::on_BTN_R15_clicked()
{
    DO_adjustY(-0.15);
}

void hqdialog_threadmill::on_BTN_R30_clicked()
{
    DO_adjustY(-0.3);
}






void hqdialog_threadmill::on_BTN_Y0_clicked()
{
    ui->LE_STEP_LY->setText("0.0");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_X0_clicked()
{
    ui->LE_STEP_LX->setText("0.0");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_3cm_clicked()
{
    ui->LE_STEP_LX->setText("0.03");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_5cm_clicked()
{
    ui->LE_STEP_LX->setText("0.05");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_10cm_clicked()
{
    ui->LE_STEP_LX->setText("0.1");
    on_BTN_WALKSTART_clicked();
}
void hqdialog_threadmill::on_BTN_13cm_clicked()
{
    ui->LE_STEP_LX->setText("0.13");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_15cm_clicked()
{
    ui->LE_STEP_LX->setText("0.15");
    on_BTN_WALKSTART_clicked();
}
void hqdialog_threadmill::on_BTN_18cm_clicked()
{
    ui->LE_STEP_LX->setText("0.18");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_LROT_clicked()
{
    ui->LE_STEP_ROT->setText("1.0");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_NOROT_clicked()
{
    ui->LE_STEP_ROT->setText("0.0");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_RROT_clicked()
{
    ui->LE_STEP_ROT->setText("-1.0");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_WALKSTOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;

        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_STOP;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}





void hqdialog_threadmill::on_BTN_21cm_clicked()
{
    ui->LE_STEP_LX->setText("0.21");
    on_BTN_WALKSTART_clicked();
}

void hqdialog_threadmill::on_BTN_25cm_clicked()
{
    ui->LE_STEP_LX->setText("0.25");
    on_BTN_WALKSTART_clicked();
}
