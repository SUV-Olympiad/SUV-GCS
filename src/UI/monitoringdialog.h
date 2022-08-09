#ifndef MONITORINGDIALOG_H
#define MONITORINGDIALOG_H

#include <QDialog>
#include <QTimer>
#include "qcustomplot.h"

namespace Ui {
class CMonitoringDialog;
}

class CManager;

class CMonitoringDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CMonitoringDialog(CManager* aManager, QWidget *parent = 0);
    ~CMonitoringDialog();

public:
    void startTimer();

private:
    void procInitGraphWidget(QCustomPlot *aCustomPlot);
    void updateGraphData(QCustomPlot *aCustomPlot, QString aName);

private Q_SLOTS:
    void updateUI();
    void onDone(int result);


private:
    Ui::CMonitoringDialog *ui;

    CManager*               mManager;
    QTimer					mTimer;

};

#endif // MONITORINGDIALOG_H
