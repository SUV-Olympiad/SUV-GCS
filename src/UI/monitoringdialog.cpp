#include "monitoringdialog.h"
#include "ui_monitoringdialog.h"
#include "manager.h"
#include "vehicle.h"

CMonitoringDialog::CMonitoringDialog(CManager* aManager, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CMonitoringDialog)
{
    ui->setupUi(this);

    connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
    mTimer.setInterval(100);

    connect(this, SIGNAL(finished(int)), this, SLOT(onDone(int)));

    mManager = aManager;


    QStringList strItemList;

    strItemList << "ROLL"
                << "PITCH"
                << "YAW"
                << "BATTERY"
                << "POSX"
                << "POSY"
                << "POSZ"
                << "POSVX"
                << "POSVY"
                << "POSVZ";

    for ( int i = 0 ; i < strItemList.size() ; i++ ) {
        ui->graph1_comboBox->addItem(strItemList[i]);
        ui->graph2_comboBox->addItem(strItemList[i]);
    }

}

CMonitoringDialog::~CMonitoringDialog()
{
    delete ui;
}

void CMonitoringDialog::startTimer()
{
    procInitGraphWidget(ui->graph1Widget);
    procInitGraphWidget(ui->graph2Widget);
    mTimer.start();
}

void CMonitoringDialog::procInitGraphWidget(QCustomPlot *aCustomPlot)
{
    QCustomPlot *customPlot = aCustomPlot;

    int graphCount = 0;

    const QColor colorList[] = {QColor(Qt::red),
                                QColor(Qt::green),
                                QColor(Qt::blue),
                                QColor(Qt::cyan),
                                QColor(Qt::magenta),
                                QColor(Qt::yellow),
                                QColor(Qt::gray),
                                QColor(Qt::lightGray),
                                QColor(Qt::darkRed),
                                QColor(Qt::darkGreen),
                                QColor(Qt::darkBlue),
                                QColor(Qt::darkCyan),
                                QColor(Qt::darkMagenta),
                                QColor(Qt::darkYellow),
                                QColor(Qt::darkGray),
                                      };


    // clear graph
    customPlot->clearGraphs();

    for ( int i = 0 ; i < mManager->numOfAgent() ; i++ ) {

        customPlot->addGraph(); // blue line
        customPlot->graph(graphCount)->setPen(QPen(colorList[i]));

        customPlot->addGraph(); // blue dot
        graphCount++;
        customPlot->graph(graphCount)->setPen(QPen(colorList[i]));
        customPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
        customPlot->xAxis->setDateTimeFormat("hh:mm:ss");
        customPlot->xAxis->setAutoTickStep(false);
        customPlot->xAxis->setTickStep(2);
        customPlot->axisRect()->setupFullAxesBox();

        // make left and bottom axes transfer their ranges to right and top axes:
        connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
        connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
    }
}

void CMonitoringDialog::updateGraphData(QCustomPlot *aCustomPlot, QString aName)
{
    Q_UNUSED(aCustomPlot);

    QString title;
    QCustomPlot *customPlot = aCustomPlot;
    QMap<int, IVehicle*> agentsMap = mManager->agents();

    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
    double value = 0;

    int graphCount = 0;

    QMap<int, IVehicle*>::iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//    foreach (IVehicle* agent, agentsMap) {
        title = aName;
        value = mManager->agent(agentsIterator.value()->id())->data(aName).toDouble();

        // set title
        ((QWidget*)customPlot->parent())->setWindowTitle(title);

        // add data to lines:
        customPlot->graph(graphCount*2+0)->addData(key, value);

        // set data of dots:
        customPlot->graph(graphCount*2+1)->clearData();
        customPlot->graph(graphCount*2+1)->addData(key, value);

        // remove data of lines that's outside visible range:
        customPlot->graph(graphCount*2+0)->removeDataBefore(key-8);

        // rescale value (vertical) axis to fit the current data:
        customPlot->graph(graphCount*2+0)->rescaleValueAxis(true);

        graphCount++;
    }

    // make key axis range scroll with the data (at a constant range size of 8):
    customPlot->xAxis->setRange(key+0.25, 4, Qt::AlignRight);
//    customPlot->yAxis->setRange(value+1.0, value-1.0, Qt::AlignRight);


    customPlot->replot();

}



void CMonitoringDialog::updateUI()
{
    updateGraphData(ui->graph1Widget, ui->graph1_comboBox->currentText());
    updateGraphData(ui->graph2Widget, ui->graph2_comboBox->currentText());
}

void CMonitoringDialog::onDone(int result)
{
    Q_UNUSED(result);
    mTimer.stop();
}

