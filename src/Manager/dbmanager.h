#ifndef DBMANAGER_H
#define DBMANAGER_H

#include <QSqlDatabase>
#include <QSqlQuery>

class dbManager
{
public:
    dbManager();

public:
    QSqlDatabase db;
};

#endif // DBMANAGER_H
