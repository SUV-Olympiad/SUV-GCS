#include "dbmanager.h"

#include <QSqlDatabase>

dbManager::dbManager()
{
    db = QSqlDatabase::addDatabase("QMYSQL");
    db.setUserName("suvlab");
    db.setPassword("suvlab");
    db.setDatabaseName("suvlab");
    db.setHostName("theseung.com");
    db.open();
}