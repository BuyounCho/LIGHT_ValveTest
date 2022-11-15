#include "huboquaddialog.h"
#include "ui_huboquaddialog.h"

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
    HUBOQUAD_AL_PDTEST,
};
HuboQuadDialog::HuboQuadDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HuboQuadDialog)
{
    ui->setupUi(this);
    AlnumHuboQuad = PODOALDialog::GetALNumFromFileName("HuboQuad");


    isWalking = false;
    joy = new RBJoystick();
    on_BTN_NarrowWalk_2_clicked();
    JoyGait = Trot;


    for(int i=0;i<12;i++){oldB[i] = 0;}
    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);
}

HuboQuadDialog::~HuboQuadDialog()
{
    delete ui;
}
double calc_td(double in)
{
    double thres = 10000;
    double MaxA = 32768;
    double td;
    if(in>thres)
    {
        td = (in-thres)/(MaxA-thres);
    }
    else if(in<-thres)
    {
        td = (in+thres)/(MaxA-thres);
    }
    else
    {
        td = 0;
    }
    return td;

}
int cnt;
void HuboQuadDialog::DisplayUpdate()
{
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
    ui->LE_NRC->setText(str.sprintf("%d", PODO_DATA.CoreSEN.IMU[0].NO_RESPONSE_CNT));
    int Mindex = 0;
    int Mnrc2 = 0;
    for(int i=0;i<12;i++)
    {
        if(PODO_DATA.CoreSEN.ENCODER[MC_GetID(i)][0].NO_RESPONSE_CNT>Mnrc2)
        {
            Mindex = i;
            Mnrc2 = PODO_DATA.CoreSEN.ENCODER[MC_GetID(i)][0].NO_RESPONSE_CNT;
        }
    }
    ui->LE_NRC_2->setText(JointNameList[MC_GetID(Mindex)]+str.sprintf(":%d",Mnrc2));

    ui->LE_NRC_3->setText(str.sprintf("%d", PODO_DATA.CoreREF.NO_RESPONSE_CNT[2]));

    cnt++;
    if(cnt%5==0)
    {
        double FrontL,LeftL,RotA;
        if(joy->connection&&ui->CB_Jcon->isChecked())
        {//connected and checked

            FrontL = calc_td(-joy->JoyAxis[1])*ui->LE_STEP_LX_2->text().toDouble();
            LeftL = calc_td(-joy->JoyAxis[0])*ui->LE_STEP_LY_2->text().toDouble();
            RotA = calc_td(-joy->JoyAxis[3])*ui->LE_STEP_ROT_2->text().toDouble();
            for(int i=0;i<12;i++)
            {
                int id = MC_GetID(i);
                int ch = MC_GetCH(i);
                if(PODO_DATA.CoreSEN.ENCODER[id][ch].Cocoa_Data.BIGERROR_ONOFF==1)
                {
                    isWalking = false;
                }
            }
            if(fabs(PODO_DATA.CoreSEN.IMU[0].Pitch)>15)
            {
                    isWalking = false;//complete fall
            }
            if(fabs(PODO_DATA.CoreSEN.IMU[0].Roll)>15)
            {
                    isWalking = false;//complete fall
            }
            if(isWalking)
            {                
                USER_COMMAND cmd;
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=FrontL;
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[1]=LeftL;
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[2]=0;
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[3]=RotA;
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=ui->LE_STEP_T_21->text().toDouble();
                if(JoyGait==Wave2)
                {
                    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=0.6;
                    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8]=0.0;//overlap
                    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9]=0.0;//dsp
                }
                if(JoyGait==Wave2+100)
                {
                    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=0.45;
                    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8]=0.5;//overlap
                    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9]=0.0;//dsp
                }
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[5]=ui->LE_STEP_FB_L->text().toDouble();
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[6]=ui->LE_STEP_LR_L->text().toDouble();
                cmd.COMMAND_DATA.USER_PARA_DOUBLE[7]=ui->LE_COM_Z_2->text().toDouble();
                cmd.COMMAND_DATA.USER_PARA_CHAR[0]=JoyGait;
                if(JoyGait==Wave2+100)
                {
                    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Wave2;
                }

                if(ui->CB_NO_ROBOT_TEST->isChecked())
                {
                    cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
                }
                else
                {
                    cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
                }
                if(ui->CB_DO_ADJUST_SLOPE->isChecked())
                {
                    cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
                }
                else
                {
                    cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
                }
                if(ui->CB_DO_FOOTZ_DIR->isChecked())
                {
                    cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
                }
                else
                {
                    cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
                }
                cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
                cmd.COMMAND_TARGET = AlnumHuboQuad;
                pLAN->SendCommand(cmd);
            }

        }
        QString str2,str3,str4;
        if(joy->connection) { str4 = "connected\n";   }
        else{ str4 = "not connected\n";  }
        str2.sprintf("BUTTONS %d%d%d%d%d%d %d%d%d%d%d%d\n"
                ,(int)joy->JoyButton[0],(int)joy->JoyButton[1],(int)joy->JoyButton[2]
                ,(int)joy->JoyButton[3],(int)joy->JoyButton[4],(int)joy->JoyButton[5]
                ,(int)joy->JoyButton[6],(int)joy->JoyButton[7],(int)joy->JoyButton[8]
                ,(int)joy->JoyButton[9],(int)joy->JoyButton[10],(int)joy->JoyButton[11]);
        str3.sprintf("AXIS\n %d \t %d \t %d \t %d \n %d \t %d \t %d \t %d"
                ,joy->JoyAxis[0],joy->JoyAxis[1]
                ,joy->JoyAxis[2],joy->JoyAxis[3]
                ,joy->JoyAxis[4],joy->JoyAxis[5]
                ,joy->JoyAxis[6],joy->JoyAxis[7]);
        str2 = str4+str2+str3;

        ui->JOYVIEW->setText(str2);
        str2.sprintf("FrontL\n%f\nLeftL\n%f\nRotA\n%f",
                     FrontL,LeftL,RotA);
        if(isWalking){str3 = "Walking"+QString().sprintf(" %d\n",JoyGait);}
        else{str3 = "Not walking\n";}
        ui->JOYVIEW_2->setText(str3+str2);

    }
    if(joy->connection&&ui->CB_Jcon->isChecked())
    {
        if(oldB[0]==false&&joy->JoyButton[0]){isWalking = true;JoyGait = Trot;}
        if(oldB[2]==false&&joy->JoyButton[2]){isWalking = true;JoyGait = Flytrot;}
        if(oldB[3]==false&&joy->JoyButton[3]){isWalking = true;JoyGait = Standing;}
        if(oldB[5]==false&&joy->JoyButton[5])
        {
            isWalking = true;
            if(JoyGait==Wave2){JoyGait = Wave2+100;}
            else{JoyGait = Wave2;}

        }
        //if(oldB[4]==false&&joy->JoyButton[4]){isWalking = true;JoyGait = Wave2+100;}
        if(oldB[1]==false&&joy->JoyButton[1])
        {
            isWalking = false;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
            if(ui->CB_NO_SAVE->isChecked())
            {
                cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
            }
            else
            {
                cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
            }
            cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_STOP;
            cmd.COMMAND_TARGET = AlnumHuboQuad;
            pLAN->SendCommand(cmd);
        }
        if(oldB[4]==false&&joy->JoyButton[4])
        {
            isWalking = false;
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_NOCON;
            cmd.COMMAND_TARGET = AlnumHuboQuad;
            pLAN->SendCommand(cmd);
            ui->CB_Jcon->setChecked(false);
        }

        for(int i=0;i<12;i++)
        {
            oldB[i] = joy->JoyButton[i];
        }
    }



}
void HuboQuadDialog::on_BTN_READYPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=ui->LE_COM_Z->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_READYPOS;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_WALKSTART_clicked()
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
    if(ui->CB_NO_ROBOT_TEST->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    if(ui->CB_DO_ADJUST_SLOPE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
    }
    if(ui->CB_DO_FOOTZ_DIR->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}
void HuboQuadDialog::on_BTN_WALKSTART_6_clicked()
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
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Wave;
    if(ui->CB_NO_ROBOT_TEST->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    if(ui->CB_DO_ADJUST_SLOPE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
    }
    if(ui->CB_DO_FOOTZ_DIR->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}
void HuboQuadDialog::on_BTN_WALKSTART_4_clicked()
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
    if(ui->CB_NO_ROBOT_TEST->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    if(ui->CB_DO_ADJUST_SLOPE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
    }
    if(ui->CB_DO_FOOTZ_DIR->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_WALKSTART_10_clicked()
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
    if(ui->CB_NO_ROBOT_TEST->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    if(ui->CB_DO_ADJUST_SLOPE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
    }
    if(ui->CB_DO_FOOTZ_DIR->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}
void HuboQuadDialog::on_BTN_WALKSTART_5_clicked()
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
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Pronk;
    if(ui->CB_NO_ROBOT_TEST->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    if(ui->CB_DO_ADJUST_SLOPE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
    }
    if(ui->CB_DO_FOOTZ_DIR->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}
void HuboQuadDialog::on_BTN_WALKSTART_7_clicked()
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
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Demo;
    if(ui->CB_NO_ROBOT_TEST->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    if(ui->CB_DO_ADJUST_SLOPE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
    }
    if(ui->CB_DO_FOOTZ_DIR->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}
void HuboQuadDialog::on_BTN_WALKSTART_8_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=ui->LE_STEP_LX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1]=ui->LE_STEP_LY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2]=ui->LE_STEP_LZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3]=ui->LE_STEP_ROT->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=ui->LE_STEP_T_4->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5]=ui->LE_STEP_FB_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6]=ui->LE_STEP_LR_L->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7]=ui->LE_COM_Z_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Flytrot;
    if(ui->CB_NO_ROBOT_TEST->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    if(ui->CB_DO_ADJUST_SLOPE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=0;
    }
    if(ui->CB_DO_FOOTZ_DIR->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_START;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}
void HuboQuadDialog::on_BTN_WALKSTART_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
    if(ui->CB_NO_SAVE->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=1;
    }
    else
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
    }
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_WALK_STOP;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}


void HuboQuadDialog::on_BTN_LOCK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_LOCK;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_NarrowWalk_clicked()
{
    Oinverse Oi;
    ui->LE_STEP_FB_L->setText(QString().sprintf("%.3f",Oi.P2HX*2));
    ui->LE_STEP_LR_L->setText(QString().sprintf("%.3f",Oi.P2HY*2+Oi.R2P));
}

void HuboQuadDialog::on_BTN_NarrowWalk_2_clicked()
{

    Oinverse Oi;
    ui->LE_STEP_FB_L->setText(QString().sprintf("%.3f",Oi.P2HX*2));
    ui->LE_STEP_LR_L->setText(QString().sprintf("%.3f",Oi.P2HY*2+Oi.R2P*2));
}

void HuboQuadDialog::on_BTN_NarrowWalk_3_clicked()
{
    Oinverse Oi;
    ui->LE_STEP_FB_L->setText(QString().sprintf("%.3f",Oi.P2HX*2));
    ui->LE_STEP_LR_L->setText(QString().sprintf("%.3f",Oi.P2HY*2));
}



void HuboQuadDialog::on_BTN_LOCK_2_clicked()
{

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_NOCON;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}




void HuboQuadDialog::on_BTN_SENSOR_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_SENSOR_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_SENSOR_FT_NULL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;     // all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_FT_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_NEW_IMU_ENABLE_clicked()
{
    // new imu enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_NEW_IMU_NULL_clicked()
{
    // new imu nulling
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_NEW_IMU_RESET_clicked()
{
    // new imu reset angle
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_NEW_IMU_RESET_2_clicked()
{
    // new imu reset to zero
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 4;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_JOY_CONNECT_clicked()
{
    if(joy->connection==false)
    {
        joy->ConnectJoy(ui->LE_JOYNAME->text());
    }
}

void HuboQuadDialog::on_BTN_JOY_DISCONNECT_clicked()
{
    ui->CB_Jcon->setChecked(false);
    joy->DisconnectJoy();

}




void HuboQuadDialog::on_BTN_W_clicked()
{
    ui->LE_STEP_T_3->setText("0.6");
    ui->LE_OVERLAP->setText("0.0");
}

void HuboQuadDialog::on_BTN_W_2_clicked()
{
    ui->LE_STEP_T_3->setText("0.45");
    ui->LE_OVERLAP->setText("0.5");
}



void HuboQuadDialog::on_BTN_PDtest_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_PDTEST;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_KP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_KD->text().toDouble();
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_PDtest_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = HUBOQUAD_AL_PDTEST;
    cmd.COMMAND_TARGET = AlnumHuboQuad;
    pLAN->SendCommand(cmd);
}

void HuboQuadDialog::on_BTN_ZERO_clicked()
{
    ui->LE_STEP_LX->setText("0.0");
    ui->LE_STEP_LY->setText("0.0");
    ui->LE_STEP_ROT->setText("0.0");
}





