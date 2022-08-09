#include "vehicle.h"
#include <Eigen/Core>
#include <QDateTime>
#include "logger.h"

IVehicle::IVehicle(QObject* parent)
    :QObject(parent)
{
}

IVehicle::IVehicle(QMap<QString, QString> aProperty, QObject *parent)
    :QObject(parent)
{
    mInfo = aProperty;

    mID = mInfo["id"].toInt();
    mScenarioStatus = INIT;
}


IVehicle::~IVehicle()
{
}

QVariant IVehicle::data(const QString aName)
{
    return data(aName.toLatin1().data());
}

QVariant IVehicle::dataROS(const QString aName)
{
    return dataROS(aName.toLatin1().data());
}

int IVehicle::id()
{
    return mID;
}

void IVehicle::setScenarioStatus(const IVehicle::ScenarioStatus aStatus)
{
    mScenarioStatus = aStatus;
}

IVehicle::ScenarioStatus IVehicle::scenarioStatus()
{
    return mScenarioStatus;
}

bool IVehicle::addInfo(const QString aName, const QString aValue, bool aOverWrite)
{
	QString name = aName.trimmed().toLower();

	if ( aOverWrite || ! mInfo.contains(name) ) {
		mInfo[name] = aValue;
        return true;
    }
    else {
        return false;
    }
}

QVariant IVehicle::info(const char *aName)
{
    if ( !mInfo.contains(QString(aName).trimmed().toLower()) ) { // Jang changed 2016. 5. 9.
        return QVariant("");
    }
    else {
        return QVariant(mInfo[QString(aName).trimmed().toLower()]);
    }
}
