#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include "qcustomplot.h"
#include "ardronegraphicsitem.h"
#include "paramdialog.h"
#include "calibdialog.h"
#include "monitoringdialog.h"
#include "emscenariodialog.h"
#include "controldialog.h"
#include "qhac_mapview.h"
#include "departurecontrol.h"

#include <QWidget>
#include <QGraphicsScene>
#include <QTimer>
#include <QThread>
#include <QLabel>
#include <QLabel>
#include <QImage>
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/photo/photo.hpp"
#include <QWebView>
#include <QWebFrame>
#include <QWebElement>
#include <QRubberBand>
#include <QApplication>
#include <QGeoCoordinate>
#include <QMutex>
#include <QListWidgetItem>

#include <rclcpp/rclcpp.hpp>


namespace Ui {
class MainWidget;
}

class CManager;
class CController;

static const QGeoCoordinate   refPos(REF_LAT, REF_LON, REF_ALT);

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MainWidget(QWidget *parent = 0);
    virtual ~MainWidget();

public:
    int                     selectVehicleId;
    bool                    processImage;
    QLabel                  *mImageLabel, *mInformationLabel, *mMapSelectionLabel;
    QGeoCoordinate getNewPositionDiff(QGeoCoordinate oldPosition, double x, double y, double z);
    static QVector3D LLH2NED(QGeoCoordinate pos);
    static QGeoCoordinate NED2LLH(QVector3D pos);
    static QGeoCoordinate ENU2LLH(QVector3D pos);
    
protected:
    bool event(QEvent * event);
    void keyEvent(QKeyEvent * event);

private:    
    void initManager();
    void subscribeROS2Topics();
    void procInitTreeWidget();
    void procInitMainPanelWidget();    
    void updateVehicleData();	
    void updateDronesInMap();
    void updateStatusText();
    void updateNotifier();
    void updateDeparture();

private:    // ROS2 Topic
    rclcpp::Node::SharedPtr _ros2node;

private Q_SLOTS:
    void updateMap();
    void updateUI();
    void runScenario();
    void stopScenario();
    void loadConfigFile();
    void checkFlight();
    void runParamDialog();
    void runCalibration();
    void runMonitoringDialog();
    void onAlarm(bool aCheckable);    
    void onControl();
    void onScenarioMode(bool aMode);
    void on_actionsendSC_triggered();

private slots:


    void on_sysList_itemClicked(QListWidgetItem *item);

private:
    Ui::MainWidget*         ui;
    QLabel*                 mRemaingTimeLabel;

    CParamDialog*           mParamDialog;
    CCalibDialog*           mCalibDialog;
    CMonitoringDialog*      mMonitorDialog;
    CEmScenarioDialog*      mEmScenarioDialog;
    CControlDialog*         mControlDialog;
    QGraphicsScene*         mMainPanelScene;
    QTimer                  mTimer;
    QTimer                  mRoadTimer;
    CManager*               mManager;
    QThread                 mManagerThread;
    QMap<int, QString>      mPrevStatusText;
    bool                    mReadyAlarm;

    qhac_mapview            *mMapView;
    DepartureControl        *mDepartureControl;
    QRubberBand             *mRubberBand;
    bool                    mRubberBandDrawing, mPolygonDrawing;
    QPoint                  mWindowPos;
    QGeoCoordinate          _base_latlng = QGeoCoordinate(36.766559, 127.281290, 82);
    QMap<int, QString>      roadList;

    float                   HEADING = 270;
    float                   TARGET_Z = 10;
    float                   DEPLOY_MINIMUM_DIST = 5.0;      // in meter
};

#endif // MAINWIDGET_H
