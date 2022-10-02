#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QXmlStreamReader>
#include <QTimer>
#include <QThread>
#include <QRect>
#include <QRectF>
#include <QVector3D>
#include <QtMath>
#include <Eigen/Dense>

class IVehicle;

class CManager : public QObject
{
    Q_OBJECT

public:
    CManager(QObject* parent = 0);
    virtual ~CManager();

public:
	int  loadAgentFile(const QString& aFilePath);
    void addAgent(const QMap<QString, QString> aAgent);

    int  numOfAgent();
    bool hasAgent(const int aID);


    IVehicle* agent(int aID);
    int groupId(int aID);
    int vehicleId(int aID);
    QMap<int, IVehicle*> agents() const;
    QMap<int, QString> agentsTime() const;
    QMap<int, int> agentsGroup() const;
    QMap<int, int> agentsVehicle() const;

    QString property(const QString& aGroup, const QString& aKey);


public Q_SLOTS:
    void onWork();
    void onTerminated();

private:
    IVehicle* createAgent(const int aID, const char* aIP);
    QMap<QString, QString> parseAgentProperties(QXmlStreamReader& aXml, int* aStatus=NULL);
    QMap<QString, QString> parseMotionCaptureProperties(QXmlStreamReader& aXml, int* aStatus=NULL);
    QMap<QString, QString> parseEmdScenProperties(QXmlStreamReader& aXml, int* aStatus=NULL);
    QMap<QString, QString> parseBaseProperties(QXmlStreamReader& aXml, int* aStatus=NULL);

private:
    QMap< QString, QString >        mEmdscen;
    QMap< QString, QString >        mBase;

    QMap<int, IVehicle*>            mAgents;
    QMap<int, QString>              mAgents_time;
    QMap<int, int>                  mAgents_group;
    QMap<int, int>                  mAgents_vehicle;
    QList<Eigen::Vector3f>          mUnknownMark;
};

#endif // DATAMANAGER_H
