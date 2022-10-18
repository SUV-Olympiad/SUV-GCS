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
#include "suvalgo.h"

#include <QMainWindow>

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
#include <QLineF>

#include <rclcpp/rclcpp.hpp>


namespace Ui {
class MainWidget;
}

class CManager;
class CController;

static const QGeoCoordinate   refPos(REF_LAT, REF_LON, REF_ALT);

class MainWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWidget(QWidget *parent = 0);
    virtual ~MainWidget();

public:
    int                     selectVehicleId;
    int                     leapState = 0;
    bool                    processImage;
    QLabel                  *mImageLabel, *mInformationLabel, *mMapSelectionLabel;
    QGeoCoordinate getNewPositionDiff(QGeoCoordinate oldPosition, double x, double y, double z);
    static QVector3D LLH2NED(QGeoCoordinate pos);
    static QGeoCoordinate NED2LLH(QVector3D pos);
    static QGeoCoordinate ENU2LLH(QVector3D pos);
    static QVector3D LLH2ENU(QGeoCoordinate pos);
    
protected:
    bool event(QEvent * event);
    void keyEvent(QKeyEvent * event);

private:    
    void initManager();
    void subscribeROS2Topics();
    void procInitTreeWidget();
    void updateVehicleData();	
    void updateDronesInMap();
    void updateStatusText();
    void updateNotifier();
    void updateDeparture();
    void showCameraPopup();

private:    // ROS2 Topic
    rclcpp::Node::SharedPtr _ros2node;

private Q_SLOTS:
    void updateMap();
    void updateUI();
    void runScenario();
    void stopScenario();
    void loadDatabase();
    void ResetDatabase();
    void loadConfigFile();
    void checkFlight();
    void runParamDialog();
    void runCalibration();
    void runMonitoringDialog();
    void onAlarm(bool aCheckable);    
    void onControl();
    void onScenarioMode(bool aMode);
    void on_actionsendSC_triggered();
    void unmannedTrafficManagement();

private slots:


    void on_sysList_itemClicked(QListWidgetItem *item);

    void on_camera_type_toggled(bool checked);

    void on_camera_type2_toggled(bool checked);

    void LeapMotion(bool checked);

    void on_leapMotionChk_toggled(bool checked);

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
    QTimer                  mUtmTimer;
    CManager*               mManager;
    QThread                 mManagerThread;
    QMap<int, QString>      mPrevStatusText;
    bool                    mReadyAlarm;

    qhac_mapview            *mMapView;
    qhac_mapview            *mMapView2;
    DepartureControl        *mDepartureControl;
    QRubberBand             *mRubberBand;
    bool                    mRubberBandDrawing, mPolygonDrawing;
    QPoint                  mWindowPos;
    QGeoCoordinate          _base_latlng = QGeoCoordinate(36.766559, 127.281290, 82);
    QMap<int, QString>      roadList;

    float                   HEADING = 270;
    float                   TARGET_Z = 10;
    float                   DEPLOY_MINIMUM_DIST = 5.0;      // in meter

    int                     camera_type = 0;


};

#endif // MAINWIDGET_H
