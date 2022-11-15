#include "PODOGUI_Base.h"
#include "ui_PODOGUI_Base.h"

#include <QMessageBox>
#include <qvector2d.h>

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>
using namespace std;

PODOGUI_Base::PODOGUI_Base(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PODOGUI_Base)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("PODOGUI_BASE");
}

PODOGUI_Base::~PODOGUI_Base()
{
    delete ui;
}
