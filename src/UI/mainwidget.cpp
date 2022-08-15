#include "mainwidget.h"
#include "ui_mainwidget.h"
#include "manager.h"
#include "sleeper.h"

#include <QKeyEvent>
#include <QFileDialog>
#include <QSignalMapper>
#include <QMessageBox>
#include <QPixmap>
#include <QImage>
#include <QtMath>

#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

MainWidget::MainWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    // init alarm
    mReadyAlarm = false;

    mRemaingTimeLabel = new QLabel("--  ");
    QFont font = mRemaingTimeLabel->font();
    font.setPointSize(25);
    font.setBold(true);
    mRemaingTimeLabel->setFont(font);
    mRemaingTimeLabel->setAlignment(Qt::AlignCenter| Qt::AlignRight);
    mRemaingTimeLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->mainToolBar->addWidget(mRemaingTimeLabel);

    mManager = new CManager();
    mParamDialog = new CParamDialog(mManager, this);
    mCalibDialog = new CCalibDialog(mManager, this);
    mMonitorDialog = new CMonitoringDialog(mManager, this);
    mEmScenarioDialog = new CEmScenarioDialog(mManager, this);
    mControlDialog = new CControlDialog(mManager, this);

    ui->mapView->init(6,6);

    // FIXME: dynamic change according to the drone position
    ui->mapView->moveByGPS(36.37501576001398, 127.35263774974969, 19);

    mMapView = ui->mapView;
    mRubberBand = NULL;
    mRubberBandDrawing = false;
    mPolygonDrawing = false;

    mImageLabel = new QLabel(this);
    mInformationLabel = new QLabel(this);
    mMapSelectionLabel = new QLabel(this);
    mImageLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    mInformationLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    mMapSelectionLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    ui->gridLayout->addWidget(mImageLabel,0,0, Qt::AlignTop);
    ui->gridLayout->addWidget(mMapSelectionLabel,0,0, Qt::AlignTop);
    ui->gridLayout->addWidget(mInformationLabel,0,0, Qt::AlignTop);

    QPalette sample_palette;
    sample_palette.setColor(QPalette::WindowText, Qt::blue);

    mImageLabel->setPalette(sample_palette);

    this->subscribeROS2Topics();
}

MainWidget::~MainWidget()
{
    mManagerThread.quit();
    if(!mManagerThread.wait(3000)) //Wait until it actually has terminated (max. 3 sec)
    {
            mManagerThread.terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
            mManagerThread.wait(); //We have to wait again here!
    }

    delete mManager;
    delete ui;
    if ( mParamDialog !=  NULL )	delete mParamDialog;
    if ( mCalibDialog !=  NULL )	delete mCalibDialog;
    delete mRemaingTimeLabel;
    if ( mEmScenarioDialog !=  NULL )	delete mEmScenarioDialog;
}

void MainWidget::initManager()
{
    mManagerThread.setObjectName("Manager");
    connect(&mManagerThread, SIGNAL(started()), mManager, SLOT(onWork()));
    connect(&mManagerThread, SIGNAL(finished()), mManager, SLOT(onTerminated()));
    mManager->moveToThread(&mManagerThread);
    mManagerThread.start();
}

void MainWidget::subscribeROS2Topics()
{
    auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    _ros2node = rclcpp::Node::make_shared("qhac3node");
}

void MainWidget::procInitTreeWidget()
{
    QStringList strItemList;

    strItemList << "MODE"
                << "ISARMED"
                << "Battery"
                << "LLH_STR"
                << "LPOS_STR";
                


    ui->treeWidget->setColumnCount(2);
    QStringList headers;
    headers << tr("Subject") << tr("Value");
    ui->treeWidget->setHeaderLabels(headers);
    ui->treeWidget->header()->resizeSection(0, 150);

    QList<QTreeWidgetItem *> items;
    int numItem = strItemList.size();
    for (int i = 0; i < numItem ; i++ ) {
        QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
        item->setText(0, strItemList[i]);
        item->setExpanded(true);

        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            QTreeWidgetItem *subitem = new QTreeWidgetItem(item);
            int sysid = agentsIterator.value()->data("SYSID").toInt();
            subitem->setText(0, QString("ID:%1[%2]").arg(agentsIterator.value()->id()).arg(sysid));
            subitem->setText(1, QString("---"));
            item->addChild(subitem);
        }        

        items.append(item);

    }

    ui->treeWidget->insertTopLevelItems(0, items);
}

void MainWidget::procInitMainPanelWidget()
{

}

void MainWidget::updateTreeData()
{
    QMap<int, IVehicle*> agentsMap = mManager->agents();

    for ( int i = 0 ; i < ui->treeWidget->topLevelItemCount() ; i++ ) {
        QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);

        int count = 0;
        QMap<int, IVehicle*>::iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            QTreeWidgetItem* subitem = item->child(count);
            QString value;
            if ( agentsIterator.value() == NULL )  {
                qDebug("Error: agent == NULL");
                continue;
            }

            value = QString("%1").arg((agentsIterator.value()->data(item->text(0))).toString());
            subitem->setText(1, value);

            if ( item->text(0) == "MONITORING_STATUS1_HEX" ) {
                QString tooltip = agentsIterator.value()->data("MONITORING_STR").toString();
                subitem->setToolTip(1, tooltip);
            }

            count++;
        }
    }
}
        
void MainWidget::updateDronesInMap()
{
    QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        IVehicle* agent = agentsIterator.value();
        QVector3D llh = agent->data("LLH").value<QVector3D>();
        float heading = agent->data("HEADING").value<qreal>();
        mMapView->updateDrone(agent->id(), llh.x(), llh.y(), heading);
    }
}

void MainWidget::updateStatusText()
{
	QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::iterator agentsIterator;

    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        int id = agentsIterator.value()->id();
		QString text = mManager->agent(id)->data("STATUSTEXT").toString();

		if ( !text.isEmpty() &&  text != mPrevStatusText[id] ) {
			QString statusText = QString("[%1] %2")
				.arg(id)
				.arg(text);

            ui->statusListWidget->addItem(statusText);
            ui->statusListWidget->scrollToBottom();
			mPrevStatusText[id] = mManager->agent(id)->data("STATUSTEXT").toString();
		}
    }
}

void MainWidget::updateNotifier()
{
    QMap<int, IVehicle*> agentsMap = mManager->agents();
    bool ready = true;

    QMap<int, IVehicle*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//    foreach (IVehicle* agent, agentsMap) {
        int id = agentsIterator.value()->id();

        if ( mManager->agent(id)->data("RTK_READY") != "YES" ) {
            ready = false;
        }
    }

}

void MainWidget::runScenario()
{
    QString scenario_name = mManager->property("emdscen", "name");
    ui->actionMode->setChecked(false);
}

void MainWidget::stopScenario()
{
    // mScenario->stop();
}

void MainWidget::loadConfigFile()
{
    // QString fileName = "/home/suv/Project/qhac4/docs/CMODEL_2EA.conf";
   QString fileName = QFileDialog::getOpenFileName(
               this,
               tr("Open Agent Configuration File"),
               QString(CONFIG_FILE_PATH),
               tr("Conf Files (*.conf)"));

    if ( !fileName.isEmpty() ) {
        mManager->loadAgentFile(fileName);

		// init manager
        initManager();

        // wait for initializing manager thread
        // TODO: reduce sleep and check init Manager is finished.
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        bool isAllAgentsReady = true;

        do{
            CSleeper::msleep(500);
            isAllAgentsReady = true;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                if(agentsIterator.value()->isInitialized == false) {
                    isAllAgentsReady = false;
                    break;
                }
            }
            qDebug() << "isAllAgentsReadey : " << isAllAgentsReady;
        } while(!isAllAgentsReady);

        procInitMainPanelWidget();
        procInitTreeWidget();        
        connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
        mTimer.setInterval(33);
        mTimer.start();

    }

    _base_latlng.setLatitude(mManager->property("base", "latitude").toDouble());
    _base_latlng.setLongitude(mManager->property("base", "longitude").toDouble());
    _base_latlng.setAltitude(mManager->property("base", "altitude").toDouble());
    qDebug() << "set base! for SITL. " << _base_latlng;
}

void MainWidget::checkFlight()
{
    const QMap<int, IVehicle*> agentsMap = mManager->agents();

    QMap<int, IVehicle*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        agentsIterator.value()->cmd("RESET_PARAM");
        agentsIterator.value()->cmd("CHECK_PARAM");
	}
}

void MainWidget::runParamDialog()
{    
    mParamDialog->updateNode();

	mParamDialog->show();
}

void MainWidget::runCalibration()
{
	mCalibDialog->initDialog();

    mCalibDialog->show();
}

void MainWidget::runMonitoringDialog()
{
    mMonitorDialog->startTimer();
    mMonitorDialog->show();
}

void MainWidget::onAlarm(bool aCheckable)
{    
    if ( aCheckable == true ) {
        mReadyAlarm = aCheckable;
    }
    else {
        // mAlarm.stop();
    }
}

void MainWidget::onControl()
{
    mControlDialog->show();
}

void MainWidget::onScenarioMode(bool aMode)
{
    QString cmdMode = "";
    if ( aMode == true ) {
        cmdMode = "OFF_EMBEDDED_SCENARIO";
    }
    else {
        cmdMode = "ON_EMBEDDED_SCENARIO";
    }

    const QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//    foreach (IVehicle* agent, agent_list) {
        agentsIterator.value()->cmd(cmdMode.toLatin1().data());
    }

}

void MainWidget::updateUI()
{	    
    updateTreeData();
    updateStatusText();
    rclcpp::spin_some(_ros2node);
    updateDronesInMap();
}

bool MainWidget::event(QEvent *event)
{    
    if ( event->type() == QEvent::KeyRelease ){
        QKeyEvent *ke = static_cast<QKeyEvent *>(event);
        this->keyEvent(ke);
    }

    return QWidget::event(event);
}

QGeoCoordinate MainWidget::getNewPositionDiff(QGeoCoordinate oldPosition, double x, double y, double z)
{
    oldPosition = oldPosition.atDistanceAndAzimuth(x, 0, z);
    oldPosition = oldPosition.atDistanceAndAzimuth(y, 90);
    return oldPosition;
}

void MainWidget::on_actionsendSC_triggered()
{
    const QMap<int, IVehicle*> agentsMap = mManager->agents();
    if (agentsMap.size() < 1) {
        QMessageBox msgBox;
        msgBox.setText("Open conf file first!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    } else {
        // mSendSCDialog->show();
    }
}

QVector3D MainWidget::LLH2NED(QGeoCoordinate pos)
{
    // Calc x,y,z of pos with refPos
    double NED_X = refPos.distanceTo(QGeoCoordinate(pos.latitude(), refPos.longitude(), refPos.altitude()));
    if (pos.latitude() < refPos.latitude())
        NED_X = -NED_X;
    double NED_Y = refPos.distanceTo(QGeoCoordinate(refPos.latitude(), pos.longitude(), refPos.altitude()));
    if (pos.longitude() < refPos.longitude())
        NED_Y = -NED_Y;
    double NED_Z = -(pos.altitude() - refPos.altitude());
    return QVector3D(NED_X, NED_Y, NED_Z);
}

QGeoCoordinate MainWidget::NED2LLH(QVector3D pos)
{
    // Calc lat, lon, alt of pos with refPos
    QGeoCoordinate LLHPosition = QGeoCoordinate(refPos.latitude(), refPos.longitude(), refPos.altitude());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.x(), 0, -pos.z());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.y(), 90);
    return LLHPosition;
}

void MainWidget::keyEvent(QKeyEvent *event)
{
	
	// find first agent
	int node = -1;
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            node = agentsIterator.value()->id();
            break;
	}
	if ( node < 0 ) return;

    QGeoCoordinate curPosition;

    switch(event->key()) {
    case Qt::Key_Q:
	{
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            agent->cmd("ARM", refPos.altitude());
            qDebug() << "ARM>>>>>>>>";
        }

	}
        break;
    case Qt::Key_W:
	{
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            agent->cmd("DISARM");
        }
	}
        break;
    case Qt::Key_A:
    {
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            qDebug() << "TAKEOFF......";
            agent->cmd("TAKEOFF", 10, HEADING);
        }
    }
        break;
    case Qt::Key_S:
    {
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            qDebug() << "LANDING......";
			agent->cmd("LANDING", HEADING);
        }
    }
        break; 
    case Qt::Key_M:
    {
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            agent->cmd("MANUAL");
        }
    }
        break;
    case Qt::Key_1:
	{
        QVector3D target_pos = QVector3D(0, 10, -10);
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            agent->cmd("MOVE_NED", target_pos, HEADING);
        }
	}
        break;
    case Qt::Key_0:
	{
        QVector3D target_pos = QVector3D(0, 0, -10);
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            agent->cmd("MOVE_NED", target_pos, HEADING);
        }
	}
        break;
    case Qt::Key_N:
	{
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            qDebug() << "MISSION_PLAN......";
            agent->cmd("MISSION_PLAN");
        }
	}
        break;
    default:
        break;
    };

    QWidget::keyPressEvent(event);
}
