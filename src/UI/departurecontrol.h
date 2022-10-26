#ifndef DEPARTURECONTROL_H
#define DEPARTURECONTROL_H

#include <QWidget>
#include <QTableWidgetItem>


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
    void showWarning(const QMap<int, QString> warning);

private:
    Ui::DepartureControl *ui;
    int warningsec = 0;

    QTableWidgetItem *newItem;
};

#endif // DEPARTURECONTROL_H
