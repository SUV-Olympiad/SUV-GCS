#include "emscenariodialog.h"
#include "ui_emscenariodialog.h"
#include "manager.h"
#include "vehicle.h"

CEmScenarioDialog::CEmScenarioDialog(CManager* aManager, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CEmScenarioDialog)
{
    ui->setupUi(this);

    ui->progressBar_config->setMinimum(0);
    ui->progressBar_config->setMaximum(100);

	ui->lineEdit_waittime->setText(QString::number(5));

    connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
    mTimer.setInterval(1000);

    connect(this, SIGNAL(finished(int)), this, SLOT(onDone(int)));

    mManager = aManager;

    mStartTime = 0.0;
    mOnReserve = false;
    mOnSetConfig = false;

}

CEmScenarioDialog::~CEmScenarioDialog()
{
    delete ui;
}

void CEmScenarioDialog::startTimer()
{
    this->updateUI();

	mTimer.start();
}

void CEmScenarioDialog::stopMusic()
{
	// mScenarioMusic.stop();
}

void CEmScenarioDialog::updateUI()
{
    QString scenario_name = "";
    QString rtk_ready = "";
	bool has_agent1 = mManager->hasAgent(1);
    double posx = 0.0;
    double posy = 0.0;
	double rot = 0.0;

    // scenario name
    scenario_name = mManager->property("emdscen", "name");
    ui->label_scenarioname->setText(scenario_name);

	// rotation for cooridnate
	rot = mManager->property("emdscen", "rot").toFloat();
	ui->label_rot->setText(QString::number(rot));

    // check rtk is ready
    if ( has_agent1 ) {
        rtk_ready = mManager->agent(1)->data("RTK_READY").toString();
    }

    // enable or disable buttons
    if ( !has_agent1 ||  rtk_ready != "YES" ) {
		ui->pushButton_reserve->setEnabled(false);
		ui->pushButton_setConfig->setEnabled(false);
    }
    else {
        ui->pushButton_reserve->setEnabled(true);
        ui->pushButton_setConfig->setEnabled(true);
    }

    // offset
    if ( has_agent1 && rtk_ready == "YES" ) {
        posx = mManager->agent(1)->data("POSX").toDouble();
        posy = mManager->agent(1)->data("POSY").toDouble();

        QString offsetx = QString("%1").arg(posx,6,'f',3);
        ui->label_offsetx->setText(offsetx);

        QString offsety = QString("%1").arg(posy,6,'f',3);
        ui->label_offsety->setText(offsety);
    }

    float gps_time = 0.0;
    int num_agent = mManager->numOfAgent();
    int num_set_conf_agent = 0;
    int num_set_start_time_agent = 0;

    const QMap<int, IVehicle*> agent_list = mManager->agents();
    QMap<int, IVehicle*>::const_iterator i;

//    foreach (IVehicle* agent, agent_list) {
    for (i = agent_list.begin(); i != agent_list.end(); ++i){
        float tow = i.value()->data("RTK_TOW").toFloat();
        if ( gps_time < tow ) {
            gps_time = tow;
        }

        bool init_embedded_sc_offset = i.value()->data("EMBEDDED_SC_OFFSET").toBool();
        bool init_embedded_sc_file = i.value()->data("EMBEDDED_SC_FILE").toBool();
        if (  !init_embedded_sc_file  || !init_embedded_sc_offset ) {
            if ( mOnSetConfig ) {
                QString path = scenario_name + "/node_" + QString::number(i.value()->id()) + ".txt";
                                i.value()->cmd("SET_SCENARIO_CONFS", posx, posy, rot, path);
            }
        }
        else {
            num_set_conf_agent++;
        }


        bool init_embedded_sc_start_time = i.value()->data("EMBEDDED_SC_START_TIME").toBool();
        if ( !init_embedded_sc_start_time ) {
            if ( mOnReserve ) {
                i.value()->cmd("RESERVE_SCENARIO_TIME", mStartTime);
            }
        }
        else {
            num_set_start_time_agent++;
        }
    }

    // gps time
    ui->label_gpstime->setText(QString::number(gps_time));

    // left time
    qint64 waitTime = ui->lineEdit_waittime->text().toInt();
    QDateTime currTime = QDateTime::currentDateTime();
    qint64 leftTime = waitTime - mReservedClickTime.secsTo(currTime);
    if ( leftTime > 0 ) {
        ui->label_lefttime->setText(QString::number(leftTime));
    }
    else {
        ui->label_lefttime->setText(QString::number(0));
    }

    // progress bar
    int config_percent = (num_set_conf_agent*100) / num_agent;
    ui->progressBar_config->setValue(config_percent);
    int starttime_percent = (num_set_start_time_agent*100) / num_agent;
    ui->progressBar_starttime->setValue(starttime_percent);

}

void CEmScenarioDialog::onDone(int result)
{
    Q_UNUSED(result);
    mTimer.stop();
}

void CEmScenarioDialog::onReserve()
{
    float gps_time = 0;
    float wait_time = ui->lineEdit_waittime->text().toFloat();

    const QMap<int, IVehicle*> agent_list = mManager->agents();
    QMap<int, IVehicle*>::const_iterator i;
//    foreach (IVehicle* agent, agent_list) {
    for (i = agent_list.begin(); i != agent_list.end(); ++i){
        // turn embedded scenario mode on
        i.value()->cmd("ON_EMBEDDED_SCENARIO");

        float tow = i.value()->data("RTK_TOW").toFloat();
        if ( gps_time < tow ) {
            gps_time = tow;
        }
    }

    mStartTime = gps_time + wait_time;

    mOnReserve =  true;

    this->updateUI();
    mReservedClickTime = QDateTime::currentDateTime();

	mMusicTimer.setSingleShot(true);
	mMusicTimer.setInterval(wait_time*1000.0);
	QTimer::singleShot(wait_time*1000.0, this, SLOT(playMusic()));

}

void CEmScenarioDialog::onSetConfig()
{
    mOnSetConfig = true;
}

void CEmScenarioDialog::onReset()
{
    // stop timer temporarily
    mTimer.stop();

    // reset internal variable
    mOnSetConfig =  false;
    mOnReserve = false;

    // send reset command
    const QMap<int, IVehicle*> agent_list = mManager->agents();
    QMap<int, IVehicle*>::const_iterator i;
//    foreach (IVehicle* agent, agent_list) {
    for (i = agent_list.begin(); i != agent_list.end(); ++i){
        // BUG: Fix me : just send the RESET_SCENARIO_CONFS for only unreset node
        bool init_embedded_sc_offset = i.value()->data("EMBEDDED_SC_OFFSET").toBool();
        bool init_embedded_sc_file = i.value()->data("EMBEDDED_SC_FILE").toBool();
        if (  init_embedded_sc_file  || init_embedded_sc_offset ) {
            i.value()->cmd("RESET_SCENARIO_CONFS");
        }
    }

    // start timer again
	mTimer.start();
}

void CEmScenarioDialog::playMusic()
{
	// mScenarioMusic.stop();

	// load music
	// QString music_path = mManager->property("emdscen", "music_path");
	// if (music_path != "") {
	// 	mScenarioMusic.setMedia(QUrl::fromLocalFile(music_path));
	// 	mScenarioMusic.setVolume(50);
	// 	mScenarioMusic.play();
	// }
}
