#include "jumpdialog.h"
#include "ui_jumpdialog.h"

#include "CommonHeader.h"

#include "BasicFiles/PODOALDialog.h"
#include "../ALPrograms/HuboQuad/Oinverse.h"

int id = 0;
int ch = 0;
double Link_Len = 0.5;
double Null_pos = 0.;
double Cur_KP = 20;
double Cur_KD = 0.3;

inline void DisplayJointReference(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentReference+Null_pos));
}
inline void DisplayJointEncoder(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentPosition+Null_pos));
}
inline void DisplayHeight(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", 2*Link_Len*sin((PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentPosition+Null_pos)*D2Rf/2)));
}
inline void DisplayJointVelocity(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentVelocity));
}
inline void DisplayJointCurrent(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSEN.ENCODER[jnum][0].CurrentCurrent));
}


enum JUMP_ALCOMMAND
{
    JUMP_AL_NO_ACT = 100,
    JUMP_AL_SET,
    JUMP_AL_READYPOS,
    JUMP_AL_JUMP_SET,
    JUMP_AL_JUMP_START,
    JUMP_AL_HOPPING_START,
    JUMP_AL_JUMP_STOP,
    JUMP_AL_PDTEST,
    JUMP_AL_SINTEST,
    JUMP_AL_TEST,
    JUMP_AL_SAVE_STOP,

    JUMP_AL_EMSTOP,
    JUMP_AL_CHANGE2Pcon,
    JUMP_AL_CHANGE2CPcon,
    JUMP_AL_CHANGE2CP_PODOcon,
    JUMP_AL_CHANGE2CP_FFcon,
    JUMP_AL_CHANGE2Ccon,
    JUMP_AL_FLYREADY,
    JUMP_AL_FLYSTART,
    JUMP_AL_SAVE_GG_STOP,
    JUMP_AL_STIFFNESSTEST,
    JUMP_AL_HYSTEST,

};

JumpDialog::JumpDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JumpDialog)
{
    ui->setupUi(this);
    AlnumJUMP = PODOALDialog::GetALNumFromFileName("JUMP");

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateJump()));
}

JumpDialog::~JumpDialog()
{
    delete ui;
}

void JumpDialog::UpdateJump(){
    QString str;
        DisplayJointReference(id, ui->LE_JUMP_REF);
        DisplayJointEncoder(id, ui->LE_JUMP_POS);
        DisplayHeight(id, ui->LE_JUMP_HEIGHT);
        DisplayJointVelocity(id, ui->LE_JUMP_VEL);
        DisplayJointCurrent(id, ui->LE_JUMP_CUR);
}
void JumpDialog::on_BTN_WALKSTART_clicked()
{

}

void JumpDialog::on_BTN_WALKSTOP_clicked()
{

}

void JumpDialog::on_BTN_PDTEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_KP_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_KD_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_PDTEST;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_PDTESTSTOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_PDTEST;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_SINTEST_clicked()
{
    USER_COMMAND cmd;

    if(ui->RB_CTRL_ON_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;
    }else if(ui->RB_CTRL_ON_CURPOS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;
    }else if(ui->RB_CTRL_ON_CURPOS_PODO->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;
    }else{
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    }

    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = 1;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_OFFSET_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_MAG_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_FREQ_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_SINTEST;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_SINSTOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_SINTEST;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_SINREADY_clicked()
{
    USER_COMMAND cmd;

    if(ui->RB_CTRL_ON_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;
    }else if(ui->RB_CTRL_ON_CURPOS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;
    }else if(ui->RB_CTRL_ON_CURPOS_PODO->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;
    }else{
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    }

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_OFFSET_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_KP_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_KD_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_READYPOS;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_PWM_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_PWM_REF->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_SET_PWM;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_EXECUTE_COMMAND_clicked()
{
    USER_COMMAND cmd;
    if(ui->RB_CTRL_ON_POS->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // PWM on
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_OFF_POS->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // PWM off
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_ON_CURPOS_PODO->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // PWM on
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_OFF_CURPOS_PODO->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // PWM off
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_ON_PWM->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // PWM on
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_OFF_PWM->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // PWM off
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_ON_CURPOS->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // PWM on
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_OFF_CURPOS->isChecked()){
            cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
            cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // PWM off
            cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
}
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_PWM_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_PWM_REF_2->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_PWM_3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_PWM_REF_3->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_PWM_4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_PWM_REF_4->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_PWM_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_BNO_JUMP_CH->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = 0.0;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_ENC_NULL_clicked()
{
    int id, ch;

    id = ui->LE_BNO_JUMP->text().toInt();
    ch = ui->LE_BNO_JUMP_CH->text().toInt();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id; // board id: 0
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch; // board ch: 0
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; //all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_RESET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_CUR_NULL_clicked()
{
    int id, ch;

    id = ui->LE_BNO_JUMP->text().toInt();
    ch = ui->LE_BNO_JUMP_CH->text().toInt();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1; // On
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CURRENT_NULLING;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_REF_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = true;     // enable
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void JumpDialog::on_BTN_REF_DISABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false;     // disalbe
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_JUMPREADY_clicked()
{
    USER_COMMAND cmd;
    double init_z = ui->LE_JUMP_INIT_POS->text().toDouble();
    double init_th = 2*asin(init_z/(2*Link_Len))*R2D-Null_pos;

    if(ui->RB_CTRL_ON_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;
    }else if(ui->RB_CTRL_ON_CURPOS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;
    }else if(ui->RB_CTRL_ON_CURPOS_PODO->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;
    }else{
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    }

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = init_th;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_KP_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_KD_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_READYPOS;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_JUMPSTART_clicked()
{
    USER_COMMAND cmd;
    if(ui->RB_CTRL_ON_POS->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;
    }
    else if(ui->RB_CTRL_ON_CURPOS->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;
    }
    else if(ui->RB_CTRL_ON_CURPOS_PODO->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;
    }
    else if(ui->RB_CTRL_ON_PWM->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
    }
    else if(ui->RB_CTRL_ON_CURPOSFF->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;
    }


    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_JUMP_FIN_VEL->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_JUMP_FIN_POS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_JUMP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_JUMP_INIT_POS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->LE_KP_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->LE_KD_JUMP->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_JUMP_START;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_JUMPSTOP_clicked()
{
    double init_z = ui->LE_JUMP_INIT_POS->text().toDouble();
    double init_th = 2*asin(init_z/(2*Link_Len))*R2D-Null_pos;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = init_th;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_JUMP_STOP;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_JUMP_SET_clicked()
{
    USER_COMMAND cmd;
    Link_Len = ui->LE_JUMP_LINK_LEN->text().toDouble();
    Null_pos = ui->LE_JUMP_NULL_POS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Link_Len;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Null_pos;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_JUMP_KFF->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_JUMP_SET;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);

}

void JumpDialog::on_BTN_TEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_BNO_JUMP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->LE_JUMP_TEST->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_TEST;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_HOPPINGSTART_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_JUMP_TIME_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_JUMP_INIT_POS_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_JUMP_FIN_POS_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_JUMP_FIN_VEL_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_JUMP_RE->text().toInt();


    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_HOPPING_START;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_SET_clicked()
{
    id = ui->LE_BNO_JUMP->text().toInt();
    ch = ui->LE_BNO_JUMP_CH->text().toInt();
    Cur_KP = ui->LE_KP_JUMP->text().toDouble();
    Cur_KD = ui->LE_KD_JUMP->text().toDouble();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = Cur_KP;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = Cur_KD;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_SET;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}



void JumpDialog::on_BTN_Chnge2_Pcon_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_CHANGE2Pcon;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_Chnge2_CPcon_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_CHANGE2CPcon;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_Chnge2_CP_PODOcon_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_CHANGE2CP_PODOcon;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_Chnge2_Ccon_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_CHANGE2Ccon;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_Chnge2_CP_FFcon_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_CHANGE2CP_FFcon;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_EMSTOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_EMSTOP;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_FLYREADY_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_JUMP_TIME_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_JUMP_INIT_POS_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_JUMP_FIN_POS_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_JUMP_FIN_VEL_2->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_FLYREADY;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_FLYSTART_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_FLYSTART;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_WARMUP_clicked()
{

    on_BTN_ENC_ENABLE_clicked();
    usleep(500*1000);
    on_BTN_REF_ENABLE_clicked();
    usleep(500*1000);
    on_BTN_CUR_ENABLE_clicked();




//    USER_COMMAND cmd;
//    // Encoder Enable
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
//    usleep(500*1000);

//    // Ref Enable
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = true;     // enable
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
//    usleep(500*1000);

//    // Current Enable
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_SIXCURRENT_ONOFF;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
//    usleep(500*1000);

//    //Encoder Reset
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // board id: all
//    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0; // board ch: 0
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_RESET;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
//    usleep(100*1000);

//    //Current Nulling
//    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;
//    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;
//    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1; // On
//    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CURRENT_NULLING;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);


}

void JumpDialog::on_BTN_StiffnessTest_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_STIFFNESSTEST;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}

void JumpDialog::on_BTN_HYSTEST_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_DATA.USER_COMMAND = JUMP_AL_HYSTEST;
    cmd.COMMAND_TARGET = AlnumJUMP;
    pLAN->SendCommand(cmd);
}
