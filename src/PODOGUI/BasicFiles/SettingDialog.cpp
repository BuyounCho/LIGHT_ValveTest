#include "SettingDialog.h"
#include "ui_SettingDialog.h"
enum _SET_JointSequentialNumber_0{
    S_RHR = 0, S_RHP, S_RKN, S_LHR, S_LHP, S_LKN,S_NUM_0
};
enum _SET_JointSequentialNumber_1{
    S_RSR = 0, S_RSP, S_REB, S_LSR, S_LSP, S_LEB, S_NUM_1
};

QString table_0_joint_name[S_NUM_0] = {
    "RHR", "RHP", "RKN",
    "LHR", "LHP", "LKN"
};
QString table_1_joint_name[S_NUM_1] = {
    "RSR", "RSP", "REB",
    "LSR", "LSP", "LEB"
};

const struct {
    int tw;
    int row;
} TW_ROW_Pairs[NO_OF_JOINTS] = {
    {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5},
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5},
};


SettingDialog::SettingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingDialog)
{
    ui->setupUi(this);


    InitTable(ui->TW_0, table_0_joint_name, S_NUM_0);\
    InitTable(ui->TW_1, table_1_joint_name, S_NUM_1);


    lastSelected = LHP;
    select_working = false;
    ChangeSelectedJoint();

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSettings()));
}

SettingDialog::~SettingDialog()
{
    delete ui;
}

void SettingDialog::UpdateSettings(){
    int mcId, mcCh, row;
    QTableWidget *tw;
    QString str;

    for(int i=0; i<NO_OF_JOINTS; i++){
//        mcId = MC_GetID(i);
//        mcCh = MC_GetCH(i);
        mcId = i;
        mcCh = 0;
        row = TW_ROW_Pairs[i].row;

        switch(TW_ROW_Pairs[i].tw){
        case 0:
            tw = ui->TW_0;
            break;
        case 1:
            tw = ui->TW_1;
            break;        
        }

        mSTAT stat = PODO_DATA.CoreSEN.MCStatus[mcId][mcCh];
        str = "";
        if(PODO_DATA.CoreSEN.ENCODER[mcId][mcCh].BoardConnection) str += "C  ";
        else                                                      str += "N  ";
        if(stat.b.HIP == 1) str += "H/";
        else                str += "-/";
        if(stat.b.RUN == 1) str += "R/";
        else                str += "-/";
        str += QString().sprintf("%d", stat.b.HME);
        tw->item(row, 0)->setText(str);
        if(stat.b.RUN == 1 && stat.b.HME == 6){
            tw->item(row, 0)->setBackgroundColor(QColor(100, 255, 100));    // green    Home(6) Run(on)
        }else if(stat.b.HME != 0){
            tw->item(row, 0)->setBackgroundColor(QColor(255, 100, 100));    // red      Home(x) Run(x)
        }else{
            tw->item(row, 0)->setBackgroundColor(QColor(255, 255, 100));    // yellow   Home(0)
        }


        str = "";
        if(stat.b.JAM == 1) str += "JAM ";
        if(stat.b.PWM == 1) str += "PWM ";
        if(stat.b.BIG == 1) str += "BIG ";
        if(stat.b.INP == 1) str += "INP ";
        if(stat.b.FLT == 1) str += "FLT ";
        if(stat.b.ENC == 1) str += "ENC ";
        if(stat.b.CUR == 1) str += "CUR ";
        if(stat.b.TMP == 1) str += "TMP ";
        if(stat.b.PS1 == 1) str += "PS1 ";
        if(stat.b.PS2 == 1) str += "PS2 ";
        if(str == ""){
            str = "-";
            tw->item(row, 1)->setBackgroundColor(QColor(255, 255, 255));
        }else{
            tw->item(row, 1)->setBackgroundColor(QColor(255, 100, 100));
        }
        tw->item(row, 1)->setText(str);

        //tw->item(row, 2)->setText(QString().sprintf("%2d", (int)PODO_DATA.CoreSEN.MCTemperature[mcId]));
        tw->item(row, 2)->setText(QString().sprintf("%2d", (int)PODO_DATA.CoreSEN.MotorTemperature[mcId][mcCh]));
        //if(PODO_DATA.CoreSEN.MCTemperature[mcId] > 60)
        if(PODO_DATA.CoreSEN.MotorTemperature[mcId][mcCh] > 60)
            tw->item(row, 2)->setBackgroundColor(QColor(255, 100, 100));
        else
            tw->item(row, 2)->setBackgroundColor(QColor(255, 255, 255));
    }
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id = lastSelected;
    ch = 0;
    //COCOA_STATUS
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.MOTOR_DIRECTION)
    {str = "Positive";}
    else
    {str = "Negative";}
    ui->LB_DIRECTION->setText(str + " direction");

    int HV= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_SEARCH_VEL;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_DIRECTION==false)
    {HV = -HV;}
    ui->LE_HOMEV2->setText(QString().sprintf("%d",HV));
    int HO= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET_DIRECTION==false)
    {HO = -HO;}
    ui->LE_HOMEO2->setText(QString().sprintf("%d",HO));
    double PL = PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.PWM_RATE_LIMIT;
    ui->LE_PWM_RATE_LIMIT_2->setText(QString().sprintf("%.2f",PL));
    double CL = PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.CURRENT_LIMIT;
    ui->LE_CURRENT_LIMIT_2->setText(QString().sprintf("%.2f",CL));

    ui->LB_FET_STATUS->setText(QString().sprintf("FET %d STATUS %d"
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FET_ONOFF
                                                 ,PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.BOARD_ACT));


    if(ui->RB_Pcon->isChecked())
    {
        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KP));
        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KI));
        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KD));

    }
    if(ui->RB_Ccon->isChecked())
    {
        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KP));
        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KI));
        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KD));

    }
    if(ui->RB_Acon->isChecked())
    {
        ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KP));
        ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KI));
        ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KD));

    }


}

void SettingDialog::InitTable(QTableWidget *table, QString j_names[], int num){
    QFont tableFont;
    tableFont.setPointSize(8);

    const int item_height = 30;
    const int item_width = 50;
    const int col_0_width = 60;
    const int col_1_width = 100;
    const int col_2_width = 30;


    // Horizontal - Column
    for(int i=0; i<3; i++){
        table->insertColumn(i);
        table->setHorizontalHeaderItem(i, new QTableWidgetItem());
        table->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->horizontalHeaderItem(i)->setFont(tableFont);
    }
    table->horizontalHeaderItem(0)->setSizeHint(QSize(col_0_width, item_height));
    table->horizontalHeaderItem(1)->setSizeHint(QSize(col_1_width, item_height));
    table->horizontalHeaderItem(2)->setSizeHint(QSize(col_2_width, item_height));
    table->setColumnWidth(0, col_0_width);
    table->setColumnWidth(1, col_1_width);
    table->setColumnWidth(2, col_2_width);
    table->horizontalHeaderItem(0)->setText("Status");
    table->horizontalHeaderItem(1)->setText("Error");
    table->horizontalHeaderItem(2)->setText("T");

    // Vertical - Row
    for(int i=0; i<num; i++){
        table->insertRow(i);
        table->setRowHeight(i,30);
        table->setVerticalHeaderItem(i, new QTableWidgetItem());
        table->verticalHeaderItem(i)->setText(j_names[i]);
        table->verticalHeaderItem(i)->setSizeHint(QSize(item_width, item_height));
        table->verticalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->verticalHeaderItem(i)->setFont(tableFont);
    }

    for(int i=0; i<num; i++){
        for(int j=0; j<3; j++){
            table->setItem(i, j, new QTableWidgetItem());
            table->item(i,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
            table->item(i,j)->setFlags(table->item(i,j)->flags() & ~Qt::ItemIsEditable);
            table->item(i,j)->setFont(tableFont);
        }
    }

    table->setMinimumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMaximumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMinimumHeight(30*(num+1) + 2);
    table->setMaximumHeight(30*(num+1) + 2);
    //table->resizeColumnsToContents();

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void SettingDialog::UnselectOtherTable(int table){
    QTableWidget *tb;

    for(int i=0; i<7; i++){
        if(i == table)
            continue;

        switch(i){
        case 0:
            tb = ui->TW_0;
            break;
        case 1:
            tb = ui->TW_1;
            break;
        }

        QList<QTableWidgetItem*> itemlist = tb->selectedItems();
        for(int j=0; j<itemlist.size(); j++){
            itemlist[j]->setSelected(false);
        }
    }
}

void SettingDialog::on_TW_0_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(0, ui->TW_0->currentRow());
    UnselectOtherTable(0);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_1_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(1, ui->TW_1->currentRow());
    UnselectOtherTable(1);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_2_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(2, ui->TW_2->currentRow());
    UnselectOtherTable(2);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_3_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(3, ui->TW_3->currentRow());
    UnselectOtherTable(3);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_4_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(4, ui->TW_4->currentRow());
    UnselectOtherTable(4);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_5_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(5, ui->TW_5->currentRow());
    UnselectOtherTable(5);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_6_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(6, ui->TW_6->currentRow());
    UnselectOtherTable(6);
    ChangeSelectedJoint();
    select_working = false;
}

int SettingDialog::FindLastSelected(int tw, int row){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(TW_ROW_Pairs[i].tw == tw && TW_ROW_Pairs[i].row == row){
            FILE_LOG(logINFO) << i;
            return i;
        }
    }
    return -1;
}
void SettingDialog::ChangeSelectedJoint(){
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id = lastSelected;
    ch = 0;
    QString str;
    str.sprintf(" (%d,%d)",id,ch);
    ui->LB_SELECTED->setText("Selected: " + JointNameList[lastSelected]+str);
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.MOTOR_DIRECTION)
    {str = "Positive";}
    else
    {str = "Negative";}
    ui->LB_DIRECTION->setText(str + " direction");
    ui->RB_Pcon->setChecked(true);
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());

    int HV= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_SEARCH_VEL;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_DIRECTION==false)
    {HV = -HV;}
    ui->LE_HOMEV2->setText(QString().sprintf("%d",HV));
    ui->LE_HOMEV->setText(ui->LE_HOMEV->text());
    int HO= PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FINDHOME_OFFSET_DIRECTION==false)
    {HO = -HO;}
    ui->LE_HOMEO2->setText(QString().sprintf("%d",HO));
    ui->LE_HOMEO->setText(ui->LE_HOMEO->text());
    double PL = PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.PWM_RATE_LIMIT;
    ui->LE_PWM_RATE_LIMIT_2->setText(QString().sprintf("%.2f",PL));
    ui->LE_PWM_RATE_LIMIT->setText(ui->LE_PWM_RATE_LIMIT_2->text());
    double CL = PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.CURRENT_LIMIT;
    ui->LE_CURRENT_LIMIT_2->setText(QString().sprintf("%.2f",CL));
    ui->LE_CURRENT_LIMIT->setText(ui->LE_CURRENT_LIMIT_2->text());

}


void SettingDialog::on_BTN_CAN_CHECK_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_INIT_CHECK_DEVICE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_FIND_HOME_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // all
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_MOVE_JOINT_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id = lastSelected;
    ch = 0;
    float time = ui->LE_MOVE_TIME->text().toFloat();
    float angle = ui->LE_MOVE_DEGREE->text().toFloat();
//    if(lastSelected == RWH || lastSelected == LWH)
//        angle /= 200.0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
//    if(lastSelected == RHAND || lastSelected == LHAND ||
//            lastSelected == RF1 || lastSelected == RF2 ||
//            lastSelected == RF3 || lastSelected == RF4 ||
//            lastSelected == LF1 || lastSelected == LF2 ||
//            lastSelected == LF3 || lastSelected == LF4)
//        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // absolute
//    else
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // relative
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = time;     // time(ms)
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = angle;	// angle
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void SettingDialog::on_BTN_SET_CURRENT_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_CURRENT_REF->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_SET_PWM_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id= lastSelected;
    ch = 0;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_PWM_REF->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_GAIN_OVERRIDE_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;
    float time = ui->LE_GO_TIME->text().toFloat();
    float gain = ui->LE_GO_GAIN->text().toFloat();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = time;
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = gain;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
void SettingDialog::on_BTN_COCOA_REQUEST_EVERYTING_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
void SettingDialog::on_BTN_COCOA_GAIN_SET_clicked()
{

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    if(ui->RB_Pcon->isChecked())
    {
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    }
    if(ui->RB_Ccon->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    }
    if(ui->RB_Acon->isChecked())
    {
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    }
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_KP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_KI->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_KD->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void SettingDialog::on_BTN_COCOA_SET_DIRECTION_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    if(PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.MOTOR_DIRECTION)
    {cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;}
    else
    {cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;}
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);

}

void SettingDialog::on_BTN_COCOA_FINDHOME_SET_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_HOMEV->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->LE_HOMEO->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_COCOA_FINDHOME_SET_2_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
void SettingDialog::on_BTN_COCOA_PWMRATE_LIM_SET_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
void SettingDialog::on_BTN_COCOA_INTERNAL_POSITION_GOTO_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_GOTO_REF->text().toInt();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_COCOA_PWMRATE_LIM_SET_2_clicked()
{

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_CURRENT_LIMIT->text().toDouble();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_EXECUTE_COMMAND_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id= lastSelected;
    ch = 0;

    USER_COMMAND cmd;
    if(ui->RB_INIT_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_ENC_ZERO->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_FET_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_FET_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_SW_COMP->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;        
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //complementary
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_SW_NON_COMP->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //non-complementary
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_FRIC_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_FRIC_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_MODE_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //position
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_MODE_PWM->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 3;     //pwm
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_ERROR_CLEAR->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;        
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // only error clear
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_JOINT_RECOVER->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // error clear + joint recovery
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_ON_CURRENT->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // ccon on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_OFF_CURRENT->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // ccon off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_ON_PWM->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // PWM on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }else if(ui->RB_CTRL_OFF_PWM->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // PWM off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_NO_ACT;
    }

    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void SettingDialog::on_RB_Pcon_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id = lastSelected;
    ch = 0;
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.P_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());
}

void SettingDialog::on_RB_Ccon_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id = lastSelected;
    ch = 0;
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.C_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());
}

void SettingDialog::on_RB_Acon_clicked()
{
    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    id = lastSelected;
    ch = 0;
    ui->LE_KP_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KP));
    ui->LE_KI_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KI));
    ui->LE_KD_2->setText(QString().sprintf("%.2f",PODO_DATA.CoreSEN.ENCODER[id][ch].HCB_Info.FOC_P_KD));
    ui->LE_KP->setText(ui->LE_KP_2->text());
    ui->LE_KI->setText(ui->LE_KI_2->text());
    ui->LE_KD->setText(ui->LE_KD_2->text());
}




