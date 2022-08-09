#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mainwidget.h"
#include "departurecontrol.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->tabWidget->addTab(new MainWidget(), QString("Traffic Control"));
    ui->tabWidget->addTab(new DepartureControl(), QString("Departure Control"));
}

MainWindow::~MainWindow()
{
    delete ui;
}
