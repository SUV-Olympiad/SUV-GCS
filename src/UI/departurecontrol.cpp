#include "departurecontrol.h"
#include "ui_departurecontrol.h"

DepartureControl::DepartureControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DepartureControl)
{
    ui->setupUi(this);
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
}

// DepartureControl::additem()
// {
//     ui->tableWidget->insertRow( ui->tableWidget->rowCount() );
// }

DepartureControl::~DepartureControl()
{
    delete ui;
}
