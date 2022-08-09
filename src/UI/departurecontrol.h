#ifndef DEPARTURECONTROL_H
#define DEPARTURECONTROL_H

#include <QWidget>

namespace Ui {
class DepartureControl;
}

class DepartureControl : public QWidget
{
    Q_OBJECT

public:
    explicit DepartureControl(QWidget *parent = nullptr);
    ~DepartureControl();

private:
    Ui::DepartureControl *ui;
};

#endif // DEPARTURECONTROL_H
