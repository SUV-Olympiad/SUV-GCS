#ifndef CCALIBDIALOG_H
#define CCALIBDIALOG_H

#include "ui_calibdialog.h"

#include <QObject>
#include <QTimer>
#include <QTableWidgetItem>

class CManager;

class CCalibDialog : public QDialog, public Ui::CalibDialog
{
	Q_OBJECT

public:
	CCalibDialog(CManager* aManager, QWidget * parent = 0);

public:
	void update();
	void initDialog();

private Q_SLOTS:
	void updateUI();
	void onGyroCalib();
	void onLevelCalib();
    void onAccelCalib();
	void onReboot();
    void cellClicked(int aRow, int aColumn);

private:
    QTableWidgetItem *searchTableItem(int aAgentID, int aColumn);

private:
	CManager*                           mManager;
	int									mNode;

	QString								mCurrCmd;
	QTimer								mTimer;


};

#endif // CCALIBDIALOG_H
