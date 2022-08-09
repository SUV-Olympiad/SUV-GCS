#ifndef PARAMCHECKWORKER_H
#define PARAMCHECKWORKER_H

#include <QObject>
#include "uav_sender.h"

class ParamcheckWorker : public QObject
{
    Q_OBJECT
public:
    ParamcheckWorker();
    ~ParamcheckWorker();
    CCModelCmdSender* mSenderCModel = nullptr;
    const QString aName;
    QMap< QString, QVariant >           mDefaultParams;

signals:

public slots:
    void process();
};

#endif // PARAMCHECKWORKER_H
