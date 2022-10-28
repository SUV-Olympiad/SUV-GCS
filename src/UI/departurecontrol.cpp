#include "departurecontrol.h"
#include "ui_departurecontrol.h"

#include "manager.h"
#include "rosdata.h"

#include <QMessageBox>
#include <QDebug>
#include <QGuiApplication>
#include <QClipboard>

DepartureControl::DepartureControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DepartureControl)
{
    ui->setupUi(this);
    ui->tableWidget->setColumnWidth(0,60);
    ui->tableWidget->setColumnWidth(1,80);
    ui->tableWidget->setColumnWidth(3,130);
    ui->tableWidget->setColumnWidth(5,220);
    ui->tableWidget->setColumnWidth(6,500);
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void DepartureControl::initData(QString data)
{
    QStringList list = data.split("~~~~~~~~");

    newItem = new QTableWidgetItem[list.size() * 8];

    for(int i=0; i<list.size() - 1; i++){
        QStringList itemList = list[i].split("\t");
        
        ui->tableWidget->insertRow(ui->tableWidget->rowCount() );
        
        newItem[i*8+0].setData(Qt::EditRole, itemList[1].toInt());
        ui->tableWidget->setItem(i,0,&newItem[i*8+0]);
        ui->tableWidget->setItem(i,1,&newItem[i*8+1]);
        ui->tableWidget->setItem(i,2,&newItem[i*8+2]);
        ui->tableWidget->setItem(i,3,&newItem[i*8+3]);
        ui->tableWidget->setItem(i,4,&newItem[i*8+4]);
        ui->tableWidget->setItem(i,5,&newItem[i*8+5]);
        ui->tableWidget->setItem(i,6,&newItem[i*8+6]);
        ui->tableWidget->setItem(i,7,&newItem[i*8+7]);
    }
}

void DepartureControl::updateData(QString data)
{
    int rowCount = ui->tableWidget->rowCount();
    QStringList list = data.split("\t");
    for(int i=0; i<rowCount; i++){
        if(newItem[i*8+0].text() == list[0]){
            newItem[i*8+1].setData(Qt::EditRole, list[1].toInt());
            newItem[i*8+2].setText(list[2]);
            newItem[i*8+3].setText(list[3]);
            newItem[i*8+4].setText(list[4]);
            newItem[i*8+5].setText(list[5]);
            newItem[i*8+6].setText(list[6]);
        }
        
    }
}

void DepartureControl::showWarning(const QMap<int, QString> warning)
{
    warningsec++;
    int rowCount = ui->tableWidget->rowCount();
    for(int i=0; i<rowCount; i++){
        int id = newItem[i*8+0].text().toInt();
        QString data;
        if(ui->tableWidget->item(i,7)){
            QString data = ui->tableWidget->item(i,7)->text();
        }
        if(warning.contains(id)){
            newItem[i*8+7].setText(warning[id]);
            if(warningsec >= 15){
                newItem[i*8+0].setBackground(QColor(255,79,40,100));
                newItem[i*8+1].setBackground(QColor(255,79,40,100));
                newItem[i*8+2].setBackground(QColor(255,79,40,100));
                newItem[i*8+3].setBackground(QColor(255,79,40,100));
                newItem[i*8+4].setBackground(QColor(255,79,40,100));
                newItem[i*8+5].setBackground(QColor(255,79,40,100));
                newItem[i*8+6].setBackground(QColor(255,79,40,100));
                newItem[i*8+7].setBackground(QColor(255,79,40,100));
            }else{
                newItem[i*8+0].setBackground(Qt::transparent);
                newItem[i*8+1].setBackground(Qt::transparent);
                newItem[i*8+2].setBackground(Qt::transparent);
                newItem[i*8+3].setBackground(Qt::transparent);
                newItem[i*8+4].setBackground(Qt::transparent);
                newItem[i*8+5].setBackground(Qt::transparent);
                newItem[i*8+6].setBackground(Qt::transparent);
                newItem[i*8+7].setBackground(Qt::transparent);
            }
        }else{
            newItem[i*8+0].setBackground(Qt::transparent);
            newItem[i*8+1].setBackground(Qt::transparent);
            newItem[i*8+2].setBackground(Qt::transparent);
            newItem[i*8+3].setBackground(Qt::transparent);
            newItem[i*8+4].setBackground(Qt::transparent);
            newItem[i*8+5].setBackground(Qt::transparent);
            newItem[i*8+6].setBackground(Qt::transparent);
            newItem[i*8+7].setBackground(Qt::transparent);
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

void DepartureControl::on_tableWidget_itemDoubleClicked(QTableWidgetItem *item)
{
    if(item->column() == 6 && item->text() != ""){
        QClipboard *clipboard = QGuiApplication::clipboard();
        clipboard->setText(item->text());
        QMessageBox msgBox;
        msgBox.setText("Copy Success!!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

