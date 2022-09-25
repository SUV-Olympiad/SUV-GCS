#ifndef VEHICLE_H
#define VEHICLE_H

#include "customconfig.h"
#include <Eigen/Core>
#include <QVariant>
#include <QPixmap>

class IVehicle : public QObject
{
    Q_OBJECT

public:
    enum ScenarioStatus { INIT, TAKEOFF, ANI, MOVING, FOLLOW, LANDING, LANDING_ON };

public:
    explicit IVehicle(QObject* parent = 0);
    explicit IVehicle(QMap<QString, QString> aProperty, QObject* parent = 0);
    virtual ~IVehicle();

public:
    /**
     * @brief agent initial function. this function is called when the Manager start.
     */
    virtual void init()=0;

    /**
     * @brief command function like ARM, DISARM, TAKEOFF, MOVE, ...
     *        you make command as string
     * @return status
     */
	virtual int  cmd(const char *aCmd, QVariant aArg1=0, QVariant aArg2=0, QVariant aArg3=0, QVariant aArg4=0, QVariant aArg5=0, QVariant aArg6=0) = 0;

    /**
     * @brief agent status information function like navdata, status data, ...
     * @param aName defined data name
     * @return agent status information (you should know the type of information)
     */
    virtual QPixmap getCamera();
    virtual QVariant data(const char* aName) = 0;
    virtual QVariant dataROS(const char* aName) = 0;

public:
    int id();
    // QPixmap getCamera();
    QVariant data(const QString aName);
    QVariant dataROS(const QString aName);

    /**
     * @brief add property of agent like gain value
     * @param aName property name
     * @param aValue property value
     * @param aOverWrite
     * @return
     */
    bool addInfo(const QString aName, const QString aValue, bool aOverWrite=false);
    /**
     * @brief get property of agent like gain value
     * @param aName
     * @return
     */
    QVariant info(const char * aName);

    bool isInitialized = false;

    // void stateCallback(const mavros_msgs::State::ConstPtr& msg);

public:
    void setScenarioStatus(const ScenarioStatus aStatus);
    ScenarioStatus scenarioStatus();

private:
    int                             mID;                    // Agent ID
    QMap< QString, QString >        mInfo;                  // Agent property
    ScenarioStatus                  mScenarioStatus;
};

#endif // VEHICLE_H
