#include "paramdialog.h"
#include "manager.h"
#include "vehicle.h"

CParamDialog::CParamDialog(CManager* aManager, QWidget * parent) :
	QDialog(parent)
{
	setupUi(this);    

    mClicked = false;
    mClickedCol = 0;
    mClickedRow = 0;
	mManager = aManager;

    connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
    mTimer.setInterval(500);

    initDialog();

    // connect signal
    connect(paramTableWidget, SIGNAL(cellChanged(int, int)), this, SLOT(tableChanged(int, int)));
    connect(paramTableWidget, SIGNAL(cellClicked(int, int)), this, SLOT(tableClicked(int, int)));
}

void CParamDialog::updateNode()
{
    QMap<int, IVehicle*> agentsMap = mManager->agents();
    updateComboBox->clear();
    QMap<int, IVehicle*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//    foreach (IVehicle* agent, agentsMap) {
        updateComboBox->addItem(QString("%1").arg(agentsIterator.value()->id()));
    }
}

void CParamDialog::tableChanged(int aRow, int aCol)
{    
    paramChangeProgressBar->setValue(0);

    int id = updateComboBox->currentText().toInt();
    if ( mManager->hasAgent(id) == false) return;

    QTableWidgetItem* name = paramTableWidget->item(aRow, 0);
    QTableWidgetItem* value = paramTableWidget->item(aRow, aCol);

    if ( aCol == mClickedCol && mClicked == true && mClickedRow == aRow && value->text() != "--") {
        if(!applyAllAgents->isChecked()) {
            IVehicle* agent = mManager->agent(id);
            QMap<QString, QVariant>  defaultParams = agent->data("DEFAULT_PARAMS").toMap();
            setParam(agent, name->text(), defaultParams[name->text()].type(), value->text());
            paramChangeProgressBar->setValue(100);
        } else {
            // Apply parameter change to all agents
            QMap<int, IVehicle*> agentsMap = mManager->agents();
            mCheckProgress = true;
            mChangedParam = name->text();
            mChangedParamValue = value->text();
            mParamChangedRetryCount = 0;

            QMap<int, IVehicle*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//            foreach (IVehicle* agent, agentsMap) {
                QMap<QString, QVariant>  defaultParams = agentsIterator.value()->data("DEFAULT_PARAMS").toMap();
                setParam(agentsIterator.value(), name->text(), defaultParams[name->text()].type(), value->text());
            }
        }
        mClicked = false;
    }

}

void CParamDialog::tableClicked(int aRow, int aCol)
{
    mClicked = true;
    mClickedCol = aCol;
    mClickedRow = aRow;
}

void CParamDialog::updateParam()
{
    int id = updateComboBox->currentText().toInt();
    if ( mManager->hasAgent(id) == false) return;

    // check parameter    
	mManager->agent(id)->cmd("RESET_PARAM");

	mManager->agent(id)->cmd("CHECK_PARAM");

    // reset all items
    while ( paramTableWidget->rowCount() > 0 )
        paramTableWidget->removeRow(0);

    IVehicle* agent = mManager->agent(id);

    QMap<QString, QVariant>  defaultParams = agent->data("DEFAULT_PARAMS").toMap();    
    QMap<QString, QVariant>  realParams = agent->data("REAL_PARAMS").toMap();

    QMapIterator<QString, QVariant> i(defaultParams);
    int num = 0;
    while ( i.hasNext() ) {
        i.next();

        paramTableWidget->insertRow(paramTableWidget->rowCount());

        QTableWidgetItem* name = new QTableWidgetItem(i.key());
        name->setFlags(name->flags() ^ Qt::ItemIsEditable);
        paramTableWidget->setItem(num, 0, name);

		QString value = QString("%1").arg(defaultParams[i.key()].toFloat());
		QTableWidgetItem* defaultValue = new QTableWidgetItem(value);
        defaultValue->setFlags(defaultValue->flags() ^ Qt::ItemIsEditable);
        paramTableWidget->setItem(num, 1, defaultValue);

        QTableWidgetItem* realValue = new QTableWidgetItem("--");
        paramTableWidget->setItem(num, 2, realValue);

        num++;
	}
}

void CParamDialog::updateUI()
{
    int id = updateComboBox->currentText().toInt();

    if ( mManager->hasAgent(id) == false) return;

    IVehicle* agent = mManager->agent(id);

    QMap<QString, QVariant>  defaultParams = agent->data("DEFAULT_PARAMS").toMap();
    QMap<QString, QVariant>  realParams = agent->data("REAL_PARAMS").toMap();

    QMapIterator<QString, QVariant> i(defaultParams);
    while ( i.hasNext() ) {
        int row = 0;
        i.next();

        QString key = i.key();
        QList<QTableWidgetItem *>  items = paramTableWidget->findItems(key, Qt::MatchExactly);
        if ( items.size() == 0 ) {
            // PASS
        }
        else if ( items.size() == 1 ) {
			bool ok;
            Qt::GlobalColor color;
            row = items[0]->row();

			QString value = QString("%1").arg(realParams[key].toFloat(&ok));
			if ( ok ) {
				paramTableWidget->item(row, 2)->setText(value);
			}
			else {
				paramTableWidget->item(row, 2)->setText("--");

				// check parameter again
//                mManager->agent(id)->cmd("CHECK_PARAM", key);
			}

			if ( realParams[key] == QString("--") ) {
				color = Qt::gray;
			}
			else if ( defaultParams[key] != realParams[key] ) {
                color = Qt::red;
            }
            else {
                color = Qt::white;
            }

            paramTableWidget->item(row,0)->setBackground(color);
            paramTableWidget->item(row,1)->setBackground(color);
            paramTableWidget->item(row,2)->setBackground(color);


        }
        else {
            qDebug("ERROR: duplicated item %s %d", key.toLatin1().data(),items.size());
            return;
        }
    }

    if (mCheckProgress) {
        QMap<int, IVehicle*> agentsMap = mManager->agents();
        int agentIndex = 0;
        int progressCount = 0;

        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//        foreach (IVehicle* agent, agentsMap) {
            QMap<QString, QVariant>  defaultParams = agentsIterator.value()->data("DEFAULT_PARAMS").toMap();
            QMap<QString, QVariant>  realParams = agentsIterator.value()->data("REAL_PARAMS").toMap();
            if (realParams[mChangedParam] == mChangedParamValue) {
                progressCount++;
            } else if (mParamChangedRetryCount <= 5) {
                setParam(agentsIterator.value(), mChangedParam, defaultParams[mChangedParam].type(), mChangedParamValue);
            }
            agentIndex++;
        }
        paramChangeProgressBar->setValue((int)((double)progressCount / agentsMap.size() * 100));

        if (progressCount == agentsMap.size()) {
            mCheckProgress = false;
        }

        mParamChangedRetryCount++;
    }

}

void CParamDialog::initDialog()
{
	paramTableWidget->setColumnCount(3);
	paramTableWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
	QStringList headers;
	headers << tr("Subject") << tr("DefaultValue") << tr("Value");
	paramTableWidget->setHorizontalHeaderLabels(headers);
	paramTableWidget->verticalHeader()->setVisible(false);

}

void CParamDialog::showEvent(QShowEvent *event)
{
    Q_UNUSED(event);
    mTimer.start();
}

void CParamDialog::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    mTimer.stop();
}

void CParamDialog::hideEvent(QHideEvent *event)
{
    Q_UNUSED(event);
    mTimer.stop();
}

void CParamDialog::on_initializeButton_clicked()
{
    int id = updateComboBox->currentText().toInt();

    if ( mManager->hasAgent(id) == false) return;

    IVehicle* agent = mManager->agent(id);

    QMap<QString, QVariant>  defaultParams = agent->data("DEFAULT_PARAMS").toMap();
    QMap<QString, QVariant>  realParams = agent->data("REAL_PARAMS").toMap();

    QMapIterator<QString, QVariant> i(defaultParams);
    while ( i.hasNext() ) {
        int row = 0;
        i.next();

        QString key = i.key();
        QString value = QString("%1").arg(defaultParams[i.key()].toFloat());
        if (realParams[key] != value) {
            setParam(agent, key, defaultParams[key].type(), value);
        }
    };

    qDebug("agent id %d's parameters are initialized.", id);
}

void CParamDialog::setParam(IVehicle *agent, QString key, QVariant::Type type, QVariant value)
{
    switch (type) {
    case QVariant::Char:
        agent->cmd("SET_PARAM", key, QVariant(value).toChar());
        break;
    case QVariant::Int:
        agent->cmd("SET_PARAM", key, QVariant(value).toInt());
        break;
    case QVariant::UInt:
        agent->cmd("SET_PARAM", key, QVariant(value).toUInt());
        break;
    case QVariant::Double:
        agent->cmd("SET_PARAM", key, QVariant(value).toDouble());
        break;
    case QMetaType::Float:
        agent->cmd("SET_PARAM", key, QVariant(value).toFloat());
        break;
    default:
        qDebug("WARN: unknown parameter %s", key.toStdString().c_str());
        break;
    }
}
