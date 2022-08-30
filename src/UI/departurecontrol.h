#ifndef DEPARTURECONTROL_H
#define DEPARTURECONTROL_H

#include <QWidget>

#include "manager.h"


namespace Ui {
class DepartureControl;
}

class DepartureControl : public QWidget
{
    Q_OBJECT

public:
    explicit DepartureControl(QWidget *parent = nullptr);
    ~DepartureControl();

    void initData(QString data);
    void updateData(QString data);


private:
    Ui::DepartureControl *ui;
};

#endif // DEPARTURECONTROL_H
