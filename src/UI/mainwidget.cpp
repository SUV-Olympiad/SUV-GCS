#include "mainwidget.h"
#include "ui_mainwidget.h"
#include "manager.h"
#include "sleeper.h"
#include "rosdata.h"
#include "departurecontrol.h"

#include <QKeyEvent>
#include <QFileDialog>
#include <QSignalMapper>
#include <QMessageBox>
#include <QPixmap>
#include <QImage>
#include <QtMath>
#include <QList>
#include <QFile>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <QSqlDatabase>
#include <QSqlQuery>

using std::placeholders::_1;

MainWidget::MainWidget(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    // init alarm
    mReadyAlarm = false;


    // mRemaingTimeLabel = new QLabel("--  ");
    // QFont font = mRemaingTimeLabel->font();
    // font.setPointSize(25);
    // font.setBold(true);
    // mRemaingTimeLabel->setFont(font);
    // mRemaingTimeLabel->setAlignment(Qt::AlignCenter| Qt::AlignRight);
    // mRemaingTimeLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    // ui->mainToolBar->addWidget(mRemaingTimeLabel);

    mManager = new CManager();
    mParamDialog = new CParamDialog(mManager, this);
    mCalibDialog = new CCalibDialog(mManager, this);
    mMonitorDialog = new CMonitoringDialog(mManager, this);
    mEmScenarioDialog = new CEmScenarioDialog(mManager, this);
    mControlDialog = new CControlDialog(mManager, this);

    ui->mapView->init(mManager, 6, 6);
    ui->mapView_2->init(mManager, 6, 6);

    // FIXME: dynamic change according to the drone position
    ui->mapView->moveByGPS(36.7721938,127.2696386, 15);
    ui->mapView_2->moveByGPS(36.7721938,127.2696386, 15);

    mMapView = ui->mapView;
    mMapView2 = ui->mapView_2;
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


    selectVehicleId = -1;
    ui->flightInfo->horizontalHeader()->setStretchLastSection(true);
    connect(ui->flightList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(on_sysList_itemClicked(QListWidgetItem*)));


    connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
    mTimer.setInterval(33);
    mTimer.start();

    connect(&mRoadTimer, SIGNAL(timeout()), this, SLOT(updateMap()));
    mRoadTimer.setInterval(200);
    mRoadTimer.start();

    ui->mapView_2->setVisible(false);
    ui->cameraData->setVisible(false);
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

//void MainWidget::showCameraPopup()
//{
//    cameraviews = new cameraview(this);
//    // cameraviews->move(QApplication::desktop()->screen()->rect().center() - cameraviews->rect().center());
//    cameraviews->show();
//}

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
    //List Add
    const QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::const_iterator agentsIterator;
    QMap<int, QColor> colorList;
    
    qsrand(time(0));
    QString departureData{""};
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        int sysid = agentsIterator.value()->data("SYSID").toInt();
        QColor color = QColor(qrand()%255, qrand()%255, qrand()%255);
        QString roadData{""};
        QString str = QString("ID : %1\tSYSID : %2").arg(agentsIterator.value()->id()).arg(sysid);
        QListWidgetItem* pItem =new QListWidgetItem(str);
        pItem->setForeground(color);
        ui->flightList->addItem(pItem);
        colorList[agentsIterator.value()->id()] = color;
        roadList[agentsIterator.value()->id()] = roadData;
        mMapView->updateColor(colorList);
        mMapView2->updateColor(colorList);


        QString str2 = QString("%1\t%2").arg(agentsIterator.value()->id()).arg(sysid);
        departureData.append(str2);
        departureData.append("//");
    }

    ui->departureControl->initData(departureData);


    //Show table
    QStringList strItemList;
    strItemList << "Vehicle Type"
                << "MODE"
                << "ISARMED"
                << "Battery"
                << "LLH_STR"
                << "LPOS_STR";
    int numItem = strItemList.size();

    for (int i = 0; i < numItem ; i++ ) {
        ui->flightInfo->insertRow(ui->flightInfo->rowCount() );
        ui->flightInfo->setItem(i,0,new QTableWidgetItem(strItemList[i]));
    }
}

void MainWidget::updateVehicleData(){
    if(selectVehicleId != -1){
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            int agentId = agentsIterator.value()->id();
            QPixmap img;
            if(agentId == selectVehicleId){
                QString type;
                if(camera_type == 0){
                    type = "FPV_CAMERA";
                }else{
                    type = "FOLLOW_CAMERA";
                }
                QString is_camtype = QString("IS_%1").arg(type);

                if(agentsIterator.value()->data(is_camtype).toBool()){
                    img = agentsIterator.value()->data(type).value<QPixmap>();
                    if(leapState == 0){
                        img = img.scaled(ui->label->width(),ui->label->height(),Qt::KeepAspectRatio);
                        ui->label->setPixmap(img);
                    }else{
                        img = img.scaled(ui->cameraData->width(),ui->cameraData->height(),Qt::KeepAspectRatio);
                        ui->cameraData->setPixmap(img);
                    }
                }else{
                    if(leapState == 0){
                        ui->label->setText("<html><head/><body><p align='center'>No camera</p></body></html>");
                    }else{
                        ui->cameraData->setText("<html><head/><body><p align='center'>No camera</p></body></html>");
                    }
                }

                for (int i = 0; i < ui->flightInfo->rowCount() ; i++ ) {
                    QString type = ui->flightInfo->item(i, 0)->text();
                    QString value;

                    if(i == 0){
                        value = mManager->vehicleName(mManager->vehicleId(agentId));
                    }else{
                        if ( agentsIterator.value() == NULL )  {
                            qDebug("Error: agent == NULL");
                            continue;
                        }
                        value = QString("%1").arg((agentsIterator.value()->data(type)).toString());
                    }
                    
                    ui->flightInfo->setItem(i,1,new QTableWidgetItem(value));
                }
            }
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
        mMapView2->updateDrone(agent->id(), llh.x(), llh.y(), heading);
    }
}

void MainWidget::updateStatusText()
{
	QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::iterator agentsIterator;
    QMap<int, QString> warningData;

    ui->statusListWidget->clear();

    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        int id = agentsIterator.value()->id();
		bool isRoute = mManager->agent(id)->data("OFFLINE").toBool();

		if (isRoute) {
			QString statusText = QString("[Group:%1\t ID:%2]\t Off path").arg(id).arg(id);

            ui->statusListWidget->addItem(statusText);
            ui->statusListWidget->scrollToBottom();
			mPrevStatusText[id] = mManager->agent(id)->data("STATUSTEXT").toString();
            warningData[id] = "Off path";
		}
    }
    ui->departureControl->showWarning(warningData);
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

void MainWidget::loadDatabase()
{
    if(mManager->numOfType() == 0){
        QMessageBox msgBox;
        msgBox.setText("Please insert vehicle type");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }else{
        mManager->getAgent();

        initManager();
        if(mManager->numOfAgent() != 0){
            procInitTreeWidget();
        }else{
            QMessageBox msgBox;
            msgBox.setText("Database is empty.");
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.exec();
        }
    }
}

void MainWidget::ResetDatabase()
{
    dbManager* mdbManger = new dbManager();
    QSqlDatabase db = mdbManger->db;

    QSqlQuery query(db);
    QString sql = QString("TRUNCATE drone");
    query.exec(sql);

    QMessageBox msgBox;
    msgBox.setText("Success! Please turn the program off and on.");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
}


void MainWidget::loadConfigFile()
{
    if(mManager->numOfType() == 0){
        QMessageBox msgBox;
        msgBox.setText("Please insert vehicle type");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }else{
        if(mManager->numOfAgent() != 0){
            QMessageBox msgBox;
            msgBox.setText("There is already an imported UAV.");
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.exec();
        }else{
            // QString fileName = "/home/suv/Project/qhac4/docs/CMODEL_2EA.conf";
            QString fileName = QFileDialog::getOpenFileName(
                    this,
                    tr("Open Agent Configuration File"),
                    QString(CONFIG_FILE_PATH),
                    tr("Conf Files (*.conf)"));

            if ( !fileName.isEmpty() ) {
                initManager();

                mManager->loadAgentFile(fileName);
                // wait for initializing manager thread
                // TODO: reduce sleep and check init Manager is finished.
                QMap<int, IVehicle*> agentsMap = mManager->agents();
                QMap<int, QString> agentsTimeMap = mManager->agentsTime();
                QMap<int, int> agentsGroupMap = mManager->agentsGroup();
                QMap<int, int> agentsVehicleMap = mManager->agentsVehicle();



                QMap<int, IVehicle*>::const_iterator agentsIterator;
                QMap<int, QString>::const_iterator agentsTimeIterator;
                QMap<int, int>::const_iterator agentsGroupIterator;
                QMap<int, int>::const_iterator agentsVehicleIterator;
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

                procInitTreeWidget();
            }


            _base_latlng.setLatitude(mManager->property("base", "latitude").toDouble());
            _base_latlng.setLongitude(mManager->property("base", "longitude").toDouble());
            _base_latlng.setAltitude(mManager->property("base", "altitude").toDouble());
            qDebug() << "set base! for SITL. " << _base_latlng;

            connect(&mRoadTimer, SIGNAL(timeout()), this, SLOT(updateMap()));
            mRoadTimer.setInterval(200);
            mRoadTimer.start();

            connect(&mUtmTimer, SIGNAL(timeout()), this, SLOT(unmannedTrafficManagement()));
            mUtmTimer.setInterval(120000);
            mUtmTimer.start();
        }
    }
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

void MainWidget::updateMap()
{
    const QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        IVehicle* agent = agentsIterator.value();
        agent->cmd("MISSION_PLAN");
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
    updateVehicleData();
    updateStatusText();
    rclcpp::spin_some(_ros2node);
    updateDronesInMap();
    updateDeparture();
    updatePointCamera();
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

void MainWidget::unmannedTrafficManagement(){
    QTime t;
    qDebug() << "[ UTM Start ]";
    const QMap<int, IVehicle*> agentsMap = mManager->agents();
    const QMap<int, int> agentsVehicleMap = mManager->agentsVehicle();

    vector<int> vehicle_tye_sets;
    vector<QLineF> mission_sets;
    vector<IVehicle*> agent_sets;
    vector<float> EE_list;
    vector<int> line_set;

    int index = 0;

    QMap<int, IVehicle*>::const_iterator agentsIterator;
    QMap<int, int>::const_iterator agentsTypeIterator;

    for (agentsTypeIterator = agentsVehicleMap.begin(); agentsTypeIterator != agentsVehicleMap.end(); ++agentsTypeIterator){
        int type = agentsTypeIterator.value();
        vehicle_tye_sets.push_back(type);
    }

    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        IVehicle* agent = agentsIterator.value();
        QList<QVariant> agent_mission = agent->data("MISSION").toList();

        if (agent_mission.isEmpty()) {
            qDebug() << agent->data("SYSID") << " vehicle don't have mission";
            return;
        }
//        QString("(lat : %1, lon : %2)").arg(start,6,'f',6).arg(end,6,'f',6)
        QPointF start, end;
        CROSData::MissionItem *fItem = agent_mission.first().value<CROSData::MissionItem*>();
        CROSData::MissionItem *lItem = agent_mission.last().value<CROSData::MissionItem*>();
        start = QPointF(fItem->lat, fItem->lon);
        end = QPointF(lItem->lat, lItem->lon);
//        qDebug() << "Start : " << QString("(lat : %1, lon : %2)").arg(fItem->lat,8,'f',8).arg(fItem->lon,8,'f',8)
//        << "  end : " << QString("(lat : %1, lon : %2)").arg(lItem->lat,8,'f',8).arg(lItem->lon,8,'f',8);
        agent_sets.push_back(agent);
        mission_sets.push_back(QLineF(start, end));
        //TODO 기체 타입별 에너지 효율 추가
        EE_list.push_back(vehicle_tye_sets[index]);
        if (agent->data("MODE") != "Auto loiter mode") {
            line_set.push_back(index);
        }

        index++;
    }

    SUVAlgo utm = SUVAlgo(mission_sets, EE_list);
    vector <vector<int>> ready_list = utm.solution(line_set);
    for (vector<int> wait_set: ready_list) {
        cout << "Ready ";
        for (int idx: wait_set) {
            cout <<agent_sets[idx]->data("SYSID").toInt() << " ";
        }
        cout << endl;
    }
    qDebug() << "Mission Start to utm";
    for (vector<int> wait_set: ready_list) {
        for (int idx: wait_set) {
            if (agent_sets[idx]->data("MODE") == "Auto loiter mode") {
                qDebug() << agent_sets[idx]->data("SYSID").toInt() << " start mission";
                agent_sets[idx]->cmd("MISSION_START");
            } else {
                qDebug() << agent_sets[idx]->data("SYSID").toInt() << " is flying ";
                continue;
            }
        }
        QTime t;
        t.start();
        while(t.elapsed()<15000)
            QCoreApplication::processEvents();
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

QGeoCoordinate MainWidget::ENU2LLH(QVector3D pos)
{
    // Calc lat, lon, alt of pos with refPos
    QGeoCoordinate LLHPosition = QGeoCoordinate(refPos.latitude(), refPos.longitude(), refPos.altitude());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.y(), 0, pos.z());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.x(), 90);
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

    // Get mission item
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
    // Start mission
    case Qt::Key_B:
	{
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            qDebug() << "MISSION_START......";
            agent->cmd("MISSION_START");
        }
	}
        break;
    // Sample use Mission
    case Qt::Key_T:
	{
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            IVehicle* agent = agentsIterator.value();
            qDebug() << "MISSION......";
            qDebug() << "Vehicle : " << agent->data("SYSID").toInt();
            QList<QVariant> list = agent->data("MISSION").toList();
            for (int i = 0; i < list.size(); ++i) {
                CROSData::MissionItem *item = list[i].value<CROSData::MissionItem*>();
                qDebug() << item->toString();
            }
        }
	}
        break;
    // 
    case Qt::Key_C:
	{
        QList<QVector3D> list0 = {QVector3D(-164.713669, 187.642624, 156.505997), QVector3D(-158.424179, 192.779358, 156.505692), 
                        QVector3D(-152.753922, 185.777817, 156.505997), QVector3D(-158.962479, 180.760254, 156.505997)};

        QList<QVector3D> list1 = {QVector3D(-38.623737, -340.648468, 150.481003), QVector3D(-46.685524, -340.951324, 150.481003), 
                        QVector3D(-46.648507, -350.015503, 150.481003), QVector3D(-38.682719, -350.037427, 150.481003)};

        QList<QVector3D> list2 = {QVector3D(1056.458130, 212.577347, 223.754000), QVector3D(1052.290039, 220.665695, 223.754000), 
                        QVector3D(1059.541138, 224.356140, 223.754000), QVector3D(1063.662109, 1063.662109, 223.753998)};

        QList<QVector3D> list3 = {QVector3D(831.557007, -274.773834, 176.599000), QVector3D(824.480774, -278.521759, 176.599000), 
                        QVector3D(820.189697, -270.640656, 176.599000), QVector3D(827.296997, -266.720276, 176.599000)};

        QList<QVector3D> list4 = {QVector3D(-370.155518, -215.225357, 187.679001), QVector3D(-363.531311, -210.761826, 187.679001), 
                        QVector3D(-368.543365, -203.241669, 187.679001), QVector3D(-375.231476, -207.735855, 187.679001)};

        // QList<QVector3D> list5 = {QVector3D(-364.25, 104.8, 0), QVector3D(-364.25, 104.8, 50), 
        //                 QVector3D(-364.25, -60.7, 50), QVector3D(-83.3, -659.6, 50),
        //                 QVector3D(154.24, -658.8, 50),QVector3D(154.9, -622, 50), QVector3D(154.9, -622, 0)};

        QList<QGeoCoordinate> res0;           
        QList<QGeoCoordinate> res1;
        QList<QGeoCoordinate> res2;
        QList<QGeoCoordinate> res3;
        QList<QGeoCoordinate> res4;
        // QList<QGeoCoordinate> res5;
        for(QVector3D point: list0){
            res0.append(ENU2LLH(point));
        }
        for(QVector3D point: list1){
            res1.append(ENU2LLH(point));
        }
        for(QVector3D point: list2){
            res2.append(ENU2LLH(point));
        }
        for(QVector3D point: list3){
            res3.append(ENU2LLH(point));
        }
        for(QVector3D point: list4){
            res4.append(ENU2LLH(point));
        }
        // for(QVector3D point: list5){
        //     res5.append(ENU2LLH(point));
        // }
        qDebug() << "iris_0\n"<<res0;
        qDebug() << "iris_1\n"<<res1;
        qDebug() << "iris_2\n"<<res2;
        qDebug() << "iris_3\n"<<res3;
        qDebug() << "iris_4\n"<<res4;
        // qDebug() << "iris_5\n"<<res5;
	}
        break;
        case Qt::Key_U:
        {
            unmannedTrafficManagement();
        }

        default:
        break;
    }; 

    QWidget::keyPressEvent(event);
}
void MainWidget::on_sysList_itemClicked(QListWidgetItem *item)
{
    QStringList list1 = item->text().split('\t');
    int id   = list1[0].replace("ID : ","").toInt();
    selectVehicleId = id;
    ui->mapView->selectVehicle(selectVehicleId);
}   


void MainWidget::updateDeparture()
{
    QStringList strItemList;
    strItemList << ""
                << "MODE"
                << "Battery"
                << "LLH_STR";
    int numItem = strItemList.size();

    const QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::const_iterator agentsIterator;
    

    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        QString departureData{""};
        QString sysid = agentsIterator.value()->data("SYSID").toString();
        departureData.append(sysid);
        departureData.append("\t");

        for (int i = 0; i < numItem ; i++ ) {
            QString value;
            if(i == 0){
                value = mManager->vehicleName(mManager->vehicleId(sysid.toInt()));
            }else{
                if ( agentsIterator.value() == NULL )  {
                    qDebug("Error: agent == NULL");
                    continue;
                }

                value = QString("%1").arg((agentsIterator.value()->data(strItemList[i])).toString());
            }

            departureData.append(value);
            departureData.append("\t");
        }

        ui->departureControl->updateData(departureData);
    }
}
void MainWidget::on_camera_type_toggled(bool checked)
{
    if(checked){
        camera_type = 0;
    }
}


void MainWidget::on_camera_type2_toggled(bool checked)
{
    if(checked){
        camera_type = 1;
    }
}

void MainWidget::on_leapMotionChk_toggled(bool checked)
{
    const QMap<int, IVehicle*> agentsMap = mManager->agents();
    QMap<int, IVehicle*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        int agentId = agentsIterator.value()->id();
        QPixmap img;
        if(agentId == selectVehicleId){
            double lat = agentsIterator.value()->data("GLOBAL_LAT").toDouble();
            double lon = agentsIterator.value()->data("GLOBAL_LON").toDouble();
            double alt = agentsIterator.value()->data("GLOBAL_ALT").toDouble();

            const QMap<int, IVehicle*> agentsMap = mManager->agents();
            if(checked){
                leapState = 1;
                ui->mapView_2->setVisible(true);
                ui->label->setVisible(false);
                ui->cameraData->setVisible(true);
                ui->mapView->setVisible(false);
            }else{
                leapState = 0;
                ui->mapView_2->setVisible(false);
                ui->label->setVisible(true);
                ui->cameraData->setVisible(false);
                ui->mapView->setVisible(true);
            }
        }
    }
}

void MainWidget::typeUpload()
{
    QMap< QString, QString >  properties;
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open Agent Configuration File"),
        QString(CONFIG_FILE_PATH),
        tr("Conf Files (*.conf)")
    );

    if ( !fileName.isEmpty() ) {
        mManager->loadVehicleFile(fileName);
        QMessageBox msgBox;
        QString str = QString("Sucess total : %1").arg(mManager->numOfType());
        msgBox.setText(str);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

void MainWidget::showMap(){
    point_camera = "";
    ui->mapView_2->setVisible(false);
    ui->label->setVisible(true);
    ui->cameraData->setVisible(false);
    ui->mapView->setVisible(true);
}

void MainWidget::showPointA()
{
    point_camera = "POINT_A_CAMERA";
    ui->mapView_2->setVisible(true);
    ui->label->setVisible(false);
    ui->cameraData->setVisible(true);
    ui->mapView->setVisible(false);
}

void MainWidget::showPointB()
{
    point_camera = "POINT_B_CAMERA";
    ui->mapView_2->setVisible(true);
    ui->label->setVisible(false);
    ui->cameraData->setVisible(true);
    ui->mapView->setVisible(false);
}

void MainWidget::showPointC()
{
    point_camera = "POINT_C_CAMERA";
    ui->mapView_2->setVisible(true);
    ui->label->setVisible(false);
    ui->cameraData->setVisible(true);
    ui->mapView->setVisible(false);
}

void MainWidget::showPointD()
{
    point_camera = "POINT_D_CAMERA";
    ui->mapView_2->setVisible(true);
    ui->label->setVisible(false);
    ui->cameraData->setVisible(true);
    ui->mapView->setVisible(false);
}

void MainWidget::showPointE()
{
    point_camera = "POINT_E_CAMERA";
    ui->mapView_2->setVisible(true);
    ui->label->setVisible(false);
    ui->cameraData->setVisible(true);
    ui->mapView->setVisible(false);
}

void MainWidget::updatePointCamera()
{
    if(mManager->numOfAgent() != 0 && point_camera != ""){
        const QMap<int, IVehicle*> agentsMap = mManager->agents();
        QMap<int, IVehicle*>::const_iterator agentsIterator;
        agentsIterator = agentsMap.begin();
        QPixmap img;
        img = agentsIterator.value()->data(point_camera).value<QPixmap>();
        img = img.scaled(ui->cameraData->width(),ui->cameraData->height(),Qt::KeepAspectRatio);
        ui->cameraData->setPixmap(img);
    }
}