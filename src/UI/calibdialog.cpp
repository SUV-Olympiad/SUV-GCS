#include "calibdialog.h"
#include "manager.h"
#include "vehicle.h"
#include <mmodel/mavlink.h>


CCalibDialog::CCalibDialog(CManager *aManager, QWidget *parent) :
	QDialog(parent)
{
	setupUi(this);

	mManager = aManager;

    connect(nodeTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(cellClicked(int, int)));

	connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
	mTimer.setInterval(100);

}

void CCalibDialog::update()
{

}

void CCalibDialog::updateUI()
{
    QTableWidgetItem* item = NULL;
    QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::iterator agentsIterator;

//    foreach (IVehicle* agent, agentsMap) {
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){

        item = this->searchTableItem(agentsIterator.value()->id(), 2);

        if ( agentsIterator.value()->data("ACK_CALIBCMD").toInt() == MAV_RESULT_ACCEPTED )	{
            item->setText(mCurrCmd + " [OK]");
        }
        else if ( agentsIterator.value()->data("ACK_REBOOT").toInt() == MAV_RESULT_ACCEPTED ) {
            item->setText("REBOOT [OK]");
        }

        // update status
        QString status = agentsIterator.value()->data("STATUSTEXT").toString();
        if ( status.startsWith("[cal]") ) {
            item = this->searchTableItem(agentsIterator.value()->id(), 3);
            item->setText(status);
        }

    }

}

void CCalibDialog::onGyroCalib()
{
        QMap<int, IVehicle*> agentsMap = mManager->agents();
	mCurrCmd = "GYRO";

	if ( comboBox->currentText().toUpper() == QString("ALL") )  {
            QMap<int, IVehicle*>::iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//            foreach (IVehicle* agent, agentsMap) {
                agentsIterator.value()->cmd("CALIB_GYRO");
                QTableWidgetItem* item = this->searchTableItem(agentsIterator.value()->id(), 2);
                item->setText("GYRO[--]");
            }
	}
	else {
		int id = comboBox->currentText().toInt();
		mManager->agent(id)->cmd("CALIB_GYRO");
        QTableWidgetItem* item = this->searchTableItem(id, 2);
        item->setText("GYRO[--]");
	}
}

void CCalibDialog::onLevelCalib()
{
	QMap<int, IVehicle*> agentsMap = mManager->agents();
	mCurrCmd = "LEVEL";

	if ( comboBox->currentText().toUpper() == QString("ALL") )  {
            QMap<int, IVehicle*>::iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//            foreach (IVehicle* agent, agentsMap) {
                agentsIterator.value()->cmd("CALIB_LEVEL");
                QTableWidgetItem* item = this->searchTableItem(agentsIterator.value()->id(), 2);
                item->setText("LEVEL[--]");
            }
	}
	else {
		int id = comboBox->currentText().toInt();
		mManager->agent(id)->cmd("CALIB_LEVEL");
        QTableWidgetItem* item = this->searchTableItem(id, 2);
        item->setText("LEVEL[--]");
    }
}

void CCalibDialog::onAccelCalib()
{
    QMap<int, IVehicle*> agentsMap = mManager->agents();
    mCurrCmd = "ACCEL";

    if ( comboBox->currentText().toUpper() == QString("ALL") )  {
        QMap<int, IVehicle*>::iterator agentsIterator;
//        foreach (IVehicle* agent, agentsMap) {
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            agentsIterator.value()->cmd("CALIB_ACCEL");
            QTableWidgetItem* item = this->searchTableItem(agentsIterator.value()->id(), 2);
            item->setText("ACCEL[--]");
        }
    }
    else {
        int id = comboBox->currentText().toInt();
        mManager->agent(id)->cmd("CALIB_ACCEL");
        QTableWidgetItem* item = this->searchTableItem(id, 2);
        item->setText("ACCEL[--]");
    }
}

void CCalibDialog::onReboot()
{
	QMap<int, IVehicle*> agentsMap = mManager->agents();
	mCurrCmd = "REBOOT";

	if ( comboBox->currentText().toUpper() == QString("ALL") )  {
            QMap<int, IVehicle*>::iterator agentsIterator;
//            foreach (IVehicle* agent, agentsMap) {
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                agentsIterator.value()->cmd("REBOOT");
                QTableWidgetItem* item = this->searchTableItem(agentsIterator.value()->id(), 2);
                item->setText("REBOOT[--]");
            }
	}
	else {
            int id = comboBox->currentText().toInt();
            mManager->agent(id)->cmd("REBOOT");
            QTableWidgetItem* item = this->searchTableItem(id, 2);
            item->setText("REBOOT[--]");
    }
}

void CCalibDialog::cellClicked(int aRow, int aColumn)
{
    Q_UNUSED(aColumn);

    QString id = nodeTableWidget->item(aRow, 0)->text();
//    comboBox->setCurrentText(id);
    comboBox->setItemText(comboBox->currentIndex(),id);
}

QTableWidgetItem *CCalibDialog::searchTableItem(int aAgentID, int aColumn)
{
    const int ID_COLUMN_NUM = 0;
    int rowCount = nodeTableWidget->rowCount();
    for ( int i = 0 ; i < rowCount ; i++ ) {
        int selID = nodeTableWidget->item(i, ID_COLUMN_NUM)->text().toInt();
        if ( aAgentID == selID ) {
            return nodeTableWidget->item(i, aColumn);
        }
    }

    return NULL;
}

void CCalibDialog::initDialog()
{
	int num = 0;

    // reset combox and table
    comboBox->clear();
    nodeTableWidget->clear();

    // reset variables
    mCurrCmd = "";

	// for combobox
	comboBox->addItem(QString("ALL"));

	// create header
	nodeTableWidget->setColumnCount(4);
	nodeTableWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
	QStringList headers;
	headers << tr("ID") << tr("SysID") << tr("Command") << tr("Status") ;
	nodeTableWidget->setHorizontalHeaderLabels(headers);
	nodeTableWidget->verticalHeader()->setVisible(false);

	// create contents
	QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//	foreach (IVehicle* agent, agentsMap) {
                int id = agentsIterator.value()->id();
		QString value = "";

		// for combobox
		comboBox->addItem(QString("%1").arg(id));

		// for nodeTableWidget
		nodeTableWidget->insertRow(nodeTableWidget->rowCount());

		value = QString("%1").arg(id);
		QTableWidgetItem* agentid = new QTableWidgetItem(value);
		agentid->setFlags(agentid->flags() ^ Qt::ItemIsEditable);
		nodeTableWidget->setItem(num, 0, agentid);

		value = mManager->agent(id)->data("SYSID").toString();
		QTableWidgetItem* sysid = new QTableWidgetItem(value);
		sysid->setFlags(sysid->flags() ^ Qt::ItemIsEditable);
		nodeTableWidget->setItem(num, 1, sysid);

		value = "";
		QTableWidgetItem* command = new QTableWidgetItem("");
		command->setFlags(command->flags() ^ Qt::ItemIsEditable);
		nodeTableWidget->setItem(num, 2, command);

		value = mManager->agent(id)->data("STATUSTEXT").toString();
		QTableWidgetItem* status = new QTableWidgetItem("");
		status->setFlags(status->flags() ^ Qt::ItemIsEditable);
		nodeTableWidget->setItem(num, 3, status);

		num++;

	}


	mTimer.start();
}
