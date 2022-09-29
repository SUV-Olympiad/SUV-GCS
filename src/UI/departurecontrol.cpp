#include "departurecontrol.h"
#include "ui_departurecontrol.h"

#include "manager.h"
#include "rosdata.h"
#include <QDebug>

DepartureControl::DepartureControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DepartureControl)
{
    ui->setupUi(this);
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void DepartureControl::initData(QString data)
{
    QStringList list = data.split("//");
    for(int i=0; i<list.size() - 1; i++){
        QStringList itemList = list[i].split("\t");
        
        ui->tableWidget->insertRow(ui->tableWidget->rowCount() );
        ui->tableWidget->setItem(i,0,new QTableWidgetItem(itemList[1]));
    }
}

void DepartureControl::updateData(QString data)
{
    int rowCount = ui->tableWidget->rowCount();
    QStringList list = data.split("\t");
    for(int i=0; i<rowCount; i++){
        if(ui->tableWidget->item(i,0)->text() == list[0]){
            ui->tableWidget->setItem(i,1,new QTableWidgetItem(list[1]));
            ui->tableWidget->setItem(i,2,new QTableWidgetItem(list[2]));
            ui->tableWidget->setItem(i,3,new QTableWidgetItem(list[3]));
        }
        
    }
}

DepartureControl::~DepartureControl()
{
    delete ui;
}
