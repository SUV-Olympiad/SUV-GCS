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

    newItem = new QTableWidgetItem[list.size() * 7];

    for(int i=0; i<list.size() - 1; i++){
        QStringList itemList = list[i].split("\t");
        
        ui->tableWidget->insertRow(ui->tableWidget->rowCount() );
        
        ui->tableWidget->setItem(i,0,new QTableWidgetItem(itemList[1]));

        ui->tableWidget->setItem(i,1,&newItem[i*7+1]);
        ui->tableWidget->setItem(i,2,&newItem[i*7+2]);
        ui->tableWidget->setItem(i,3,&newItem[i*7+3]);
        ui->tableWidget->setItem(i,4,&newItem[i*7+4]);
        ui->tableWidget->setItem(i,5,&newItem[i*7+5]);
        ui->tableWidget->setItem(i,6,&newItem[i*7+6]);
    }
}

void DepartureControl::updateData(QString data)
{
    int rowCount = ui->tableWidget->rowCount();
    QStringList list = data.split("\t");
    for(int i=0; i<rowCount; i++){
        if(ui->tableWidget->item(i,0)->text() == list[0]){
            newItem[i*7+1].setText(list[1]);
            newItem[i*7+2].setText(list[2]);
            newItem[i*7+3].setText(list[3]);
            newItem[i*7+4].setText(list[4]);
        }
        
    }
}

void DepartureControl::showWarning(const QMap<int, QString> warning)
{
    warningsec++;
    int rowCount = ui->tableWidget->rowCount();
    for(int i=0; i<rowCount; i++){
        int id = ui->tableWidget->item(i,0)->text().toInt();
        QString data;
        if(ui->tableWidget->item(i,6)){
            QString data = ui->tableWidget->item(i,6)->text();
        }
        if(warning.contains(id)){
            newItem[i*7+6].setText(warning[id]);
            if(warningsec >= 15){
                ui->tableWidget->item(i,0)->setBackground(QColor(255,79,40,100));
                newItem[i*7+1].setBackground(QColor(255,79,40,100));
                newItem[i*7+2].setBackground(QColor(255,79,40,100));
                newItem[i*7+3].setBackground(QColor(255,79,40,100));
                newItem[i*7+4].setBackground(QColor(255,79,40,100));
                newItem[i*7+5].setBackground(QColor(255,79,40,100));
                newItem[i*7+6].setBackground(QColor(255,79,40,100));
            }else{
                ui->tableWidget->item(i,0)->setBackground(Qt::transparent);
                newItem[i*7+1].setBackground(Qt::transparent);
                newItem[i*7+2].setBackground(Qt::transparent);
                newItem[i*7+3].setBackground(Qt::transparent);
                newItem[i*7+4].setBackground(Qt::transparent);
                newItem[i*7+5].setBackground(Qt::transparent);
                newItem[i*7+6].setBackground(Qt::transparent);
            }
        }
    }
    if(warningsec == 30){
        warningsec = 0;
    }
}

DepartureControl::~DepartureControl()
{
    delete ui;
}
