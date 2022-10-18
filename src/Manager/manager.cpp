#include "manager.h"
#include "UAV/uav.h"
#include "vehicle.h"

#include <QFile>
#include <QDateTime>

#include "dbmanager.h"
#include <QSqlDatabase>
#include <QSqlQuery>

CManager::CManager(QObject *parent) :
    QObject(parent)
{
    mdbManger = new dbManager();
    db = mdbManger->db;
}

CManager::~CManager()
{
}

void CManager::onWork()
{
    const QMap<int, IVehicle*> agents = this->agents();
    QMap<int, IVehicle*>::const_iterator i;
    for (i = agents.begin(); i != agents.end(); ++i){
        if (i.value() != NULL){
            i.value()->init();
        }
    }
}

void CManager::onTerminated()
{ 
    QMap<int, IVehicle*>::iterator i;
    for (i = mAgents.begin(); i != mAgents.end(); ++i) {
        delete i.value();
    }
}

int CManager::loadVehicleFile(const QString &aFilePath)
{    
    QFile* file = new QFile(aFilePath);

    if (!file->open(QIODevice::ReadOnly | QIODevice::Text)) {
        return -1;
    }

    QXmlStreamReader xml(file);
    QMap< QString, QString >  properties;
    QMap< QString, QString >  mc;    
    while(!xml.atEnd() && !xml.hasError()) {        
        QXmlStreamReader::TokenType token = xml.readNext();

        // If token is just StartDocument, we'll go to next
        if(token == QXmlStreamReader::StartDocument) {
            continue;
        }


        // If token is StartElement, we'll see if we can read it
        if(token == QXmlStreamReader::StartElement) {
            if(xml.name() == "types") {
                continue;
            }
            else if(xml.name() == "type") {                
                properties = this->parseAgentProperties(xml);
                // check necessary elements
				if ( ! properties.contains("id") || !properties.contains("image") || !properties.contains("name") ) {
					qDebug("ERROR :  require necessary elements (id and image) ");
                    continue;
                }
                this->addVehicle(properties);
            }
            else if ( xml.name() == "emdscen") {
                mEmdscen = this->parseEmdScenProperties(xml);
            }
            else if ( xml.name() == "base") {
                mBase = this->parseBaseProperties(xml);
            }
        }
    }

    if(xml.hasError()) {
		qDebug("ERROR : There are errors in conf file %s", xml.errorString().toLatin1().data());
        return -1;
    }

    xml.clear();
    delete file;

    return 0;
}

int CManager::loadAgentFile(const QString &aFilePath)
{    
    QFile* file = new QFile(aFilePath);

    if (!file->open(QIODevice::ReadOnly | QIODevice::Text)) {
        return -1;
    }

    QXmlStreamReader xml(file);
    QMap< QString, QString >  properties;
    QMap< QString, QString >  mc;    
    while(!xml.atEnd() && !xml.hasError()) {        
        QXmlStreamReader::TokenType token = xml.readNext();

        // If token is just StartDocument, we'll go to next
        if(token == QXmlStreamReader::StartDocument) {
            continue;
        }


        // If token is StartElement, we'll see if we can read it
        if(token == QXmlStreamReader::StartElement) {
            if(xml.name() == "agents") {
                continue;
            }
            else if(xml.name() == "agent") {                
                properties = this->parseAgentProperties(xml);
                // check necessary elements
				if ( ! properties.contains("id") || !properties.contains("type") || !properties.contains("group") || !properties.contains("vehicle") ) {
					qDebug("ERROR :  require necessary elements (id and type and group and vehicle) ");
                    continue;
                }


                this->addAgent(properties);
            }
            else if ( xml.name() == "emdscen") {
                mEmdscen = this->parseEmdScenProperties(xml);
            }
            else if ( xml.name() == "base") {
                mBase = this->parseBaseProperties(xml);
            }
        }
    }

    if(xml.hasError()) {
		qDebug("ERROR : There are errors in conf file %s", xml.errorString().toLatin1().data());
        return -1;
    }

    xml.clear();
    delete file;

    return 0;
}

void CManager::getAgent(){
    QSqlQuery query(db);
    QString sql = QString("select * from drone");
    query.exec(sql);
    while(query.next()){
        QMap<QString, QString> data;
        data["id"] = query.value(1).toString();
        data["sysid"] = query.value(2).toString();
        data["type"] = query.value(3).toString();
        data["group"] = query.value(4).toString();
        data["vehicle"] = query.value(5).toString();
        this->addAgent(data);
    }
}

void CManager::addVehicle(const QMap<QString, QString> aProperty)
{
    int id = aProperty["id"].toInt();
    QString image = aProperty["image"];
    QString name = aProperty["name"];
    mAgents_vehicle_type_name.insert(id, name);
    mAgents_vehicle_type_image.insert(id, image);

}

void CManager::addAgent(const QMap<QString, QString> aProperty)
{
    IVehicle* agent = nullptr;

    QString type = aProperty["type"];
    int id = aProperty["id"].toInt();
    int sysid = aProperty["sysid"].toInt();
    int group = aProperty["group"].toInt();
    int vehicle = aProperty["vehicle"].toInt();
    QString time = aProperty["time"];

    if(!this->hasAgent(id)){
        // create agent object
        if ( type == "CMODEL" ) {
            agent = new CUAV(aProperty, this);
            agent->init();
        }
        else {
            qDebug("ERROR : Cannot support the %s", type.toLatin1().data());
            return;
        }

        //DB 추가
        QSqlQuery query(db);

        QString sql = QString("select * from drone where id='%1'").arg(id);
        query.exec(sql);
        query.first();
        int count = query.value(0).toInt();
        if(count == 0){
            sql = QString("INSERT INTO drone (id,sysid,type,groupId,vehicle)VALUES('%1','%2','%3','%4','%5')").arg(id).arg(sysid).arg(type).arg(group).arg(vehicle);
        }else{
            sql = QString("UPDATE drone SET type='%1', sysid='%2', groupId='%3',vehicle='%4' WHERE id='%5'").arg(type).arg(sysid).arg(group).arg(vehicle).arg(id);
        }
        query.exec(sql);

        // insert agent object to manager
        mAgents.insert(id, agent);
        mAgents_group.insert(id, group);
        mAgents_vehicle.insert(id, vehicle);
        mAgents_time.insert(id, time);
    }
}


int CManager::numOfAgent()
{
    return mAgents.size();
}

bool CManager::hasAgent(const int aID)
{
    return mAgents.contains(aID);
}

IVehicle *CManager::agent(int aID)
{
    if ( mAgents.contains(aID)) {
        return mAgents[aID];
    }
    else {
        qDebug("ERROR : cannot find agent (ID:%d)", aID);
        return NULL;
    }
}

QString CManager::vehicleImage(int aID)
{
    if ( mAgents_vehicle_type_image.contains(aID)) {
        return mAgents_vehicle_type_image[aID];
    }
    else {
        qDebug("ERROR : cannot find vehicleImage (ID:%d)", aID);
        return NULL;
    }
}

QString CManager::vehicleName(int aID)
{
    if ( mAgents_vehicle_type_name.contains(aID)) {
        return mAgents_vehicle_type_name[aID];
    }
    else {
        qDebug("ERROR : cannot find vehicleName (ID:%d)", aID);
        return NULL;
    }
}

int CManager::groupId(int aID)
{
    if ( mAgents_group.contains(aID)) {
        return mAgents_group[aID];
    }
    else {
        qDebug("ERROR : cannot find agent (ID:%d)", aID);
        return NULL;
    }
}

int CManager::vehicleId(int aID)
{
    if ( mAgents_vehicle.contains(aID)) {
        return mAgents_vehicle[aID];
    }
    else {
        qDebug("ERROR : cannot find agent (ID:%d)", aID);
        return NULL;
    }
}

QMap<int, IVehicle *> CManager::agents() const
{
    return mAgents;
}

QMap<int, QString> CManager::agentsTime() const
{
    return mAgents_time;
}

QMap<int, int> CManager::agentsGroup() const
{
    return mAgents_group;
}

QMap<int, int> CManager::agentsVehicle() const
{
    return mAgents_vehicle;
}


QString CManager::property(const QString &aGroup, const QString &aKey)
{
    if ( aGroup.toLower().trimmed() == "emdscen" ) {
        return mEmdscen[aKey];
    } else if ( aGroup.toLower().trimmed() == "base" ) {
        return mBase[aKey];
    } 
}

QMap<QString, QString> CManager::parseAgentProperties(QXmlStreamReader &aXml, int* aStatus)
{
    QMap<QString, QString> property;

    // Let's check that we're really getting a agent
    if(aXml.tokenType() != QXmlStreamReader::StartElement &&
            aXml.name() == "agent") {
        if ( aStatus != NULL ) *aStatus = -1;
        return property;
    }

    // Let's get the attributes for agent
    QXmlStreamAttributes attributes = aXml.attributes();
    if(attributes.hasAttribute("id")) {
        property["id"] = attributes.value("id").toString().trimmed();
    }
    aXml.readNext();

    while(!(aXml.tokenType() == QXmlStreamReader::EndElement && aXml.name() == "agent")) {
        if(aXml.tokenType() == QXmlStreamReader::StartElement) {
            QStringRef name = aXml.name();
            aXml.readNext();
            property[name.toString().trimmed().toLower()] = aXml.text().toString().trimmed(); // Jang added 2016. 5. 9.
        }
        aXml.readNext();
    }

    return property;
}


QMap<QString, QString> CManager::parseEmdScenProperties(QXmlStreamReader &aXml, int *aStatus)
{
    QMap<QString, QString> property;

    // Let's check that we're really getting a agent
    if(aXml.tokenType() != QXmlStreamReader::StartElement &&
            aXml.name() == "emdscen") {
        if ( aStatus != NULL ) *aStatus = -1;
        return property;
    }


    while(!(aXml.tokenType() == QXmlStreamReader::EndElement && aXml.name() == "emdscen")) {
        if(aXml.tokenType() == QXmlStreamReader::StartElement) {
            QStringRef name = aXml.name();
            aXml.readNext();
            property[name.toString()] = aXml.text().toString().trimmed();            
        }
        aXml.readNext();
    }
    return property;
}

QMap<QString, QString> CManager::parseBaseProperties(QXmlStreamReader &aXml, int *aStatus)
{
    QMap<QString, QString> property;

    // Let's check that we're really getting a agent
    if(aXml.tokenType() != QXmlStreamReader::StartElement &&
            aXml.name() == "base") {
        if ( aStatus != NULL ) *aStatus = -1;
        return property;
    }

    while(!(aXml.tokenType() == QXmlStreamReader::EndElement && aXml.name() == "base")) {
        if(aXml.tokenType() == QXmlStreamReader::StartElement) {
            QStringRef name = aXml.name();
            aXml.readNext();
            property[name.toString()] = aXml.text().toString().trimmed();
        }
        aXml.readNext();
    }
    return property;
}


// QList<QVector3D> CManager::getAgentsPositionsToRect(QRectF qrect)
// {
//     QList<QVector3D> targetPositions;

//     float newZ = 5;

//     int numTargetPositions = this->numOfAgent()/2;
//     for(int i=1;i<=numTargetPositions;i++) {
//         if (qFabs(qrect.width()) > qFabs(qrect.height())) {
//             targetPositions.push_back(QVector3D(qrect.topLeft().x() + i*qrect.width()/(numTargetPositions+1), qrect.topLeft().y() + qrect.height()*1/2, newZ));
//         } else {
//             targetPositions.push_back(QVector3D(qrect.topLeft().x() + qrect.width()/2, qrect.topLeft().y() + i*qrect.height()/(numTargetPositions+1), newZ));
//         }
//     }

//     return targetPositions;
// }

// void CManager::moveAgentsToRect(QRect qrect)
// {
//     QMap<int, QVector3D*> targetPositions;

//     // TODO : Calculate Target Positions for existing agents..

//     float HFOV = 69.4;
//     float VFOV = 42.5;

//     float newZHFOV = qrect.width()/(4 * qTan(HFOV/2 * M_PI / 180));
//     float newZVFOV = qrect.height()/(4 * qTan(VFOV/2 * M_PI / 180));
//     float newZ = (newZHFOV > newZVFOV ? newZHFOV : newZVFOV) + 1;

//     targetPositions.insert(1, new QVector3D(qrect.topLeft().x() + qrect.width()/4,
//                                             qrect.topLeft().y() + qrect.height()/4, newZ));
//     targetPositions.insert(2, new QVector3D(qrect.topLeft().x() + qrect.width()*3/4,
//                                             qrect.topLeft().y() + qrect.height()/4, newZ));
//     targetPositions.insert(3, new QVector3D(qrect.topLeft().x() + qrect.width()/4,
//                                             qrect.topLeft().y() + qrect.height()*3/4, newZ));
//     targetPositions.insert(4, new QVector3D(qrect.topLeft().x() + qrect.width()*3/4,
//                                             qrect.topLeft().y() + qrect.height()*3/4, newZ));


//     // QList<QVector3D*>positionOffsets;  // For simulation env.
//     QMap<int, QVector3D*> positionOffsets;  // For simulation env.

//     positionOffsets.insert(1, new QVector3D(20, 20, 0));
//     positionOffsets.insert(2, new QVector3D(-20, 20, 0));
//     positionOffsets.insert(3, new QVector3D(20, -20, 0));
//     positionOffsets.insert(4, new QVector3D(-20, -20, 0));

//     // TODO : Generate trajectory to Target Positions..

//     QMap<int, IVehicle*>::const_iterator agentsIterator;
//     for (agentsIterator = mAgents.begin(); agentsIterator != mAgents.end(); ++agentsIterator){
//         int agentId = agentsIterator.value()->id();
//         printf("Agent id : %d, move to (%.2lf, %.2lf, %.2lf)\n", agentId,
//                 targetPositions[agentId]->x(), targetPositions[agentId]->y(), targetPositions[agentId]->z());
//         this->agent(agentId)->cmd("MOVE", targetPositions[agentId]->x() + positionOffsets[agentId]->x(),
//              targetPositions[agentId]->y() + positionOffsets[agentId]->y(),
//              targetPositions[agentId]->z() + positionOffsets[agentId]->z(),
//              0);
//     }
// }
