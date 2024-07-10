#pragma execution_character_set("utf-8")
#include "frmvideopanel.h"
#include "ui_frmvideopanel.h"
#include "camerasetdialog.h"



frmVideoPanel::frmVideoPanel(QWidget *parent) : QWidget(parent), ui(new Ui::frmVideoPanel)
{
    ui->setupUi(this);

}

frmVideoPanel::~frmVideoPanel()
{
    delete ui;
}

void frmVideoPanel::mtimeout()
{

}

