#include "departurecontrol.h"
#include "ui_departurecontrol.h"

DepartureControl::DepartureControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DepartureControl)
{
    ui->setupUi(this);
}

DepartureControl::~DepartureControl()
{
    delete ui;
}
