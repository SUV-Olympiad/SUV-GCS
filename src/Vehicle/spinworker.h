#ifndef SPINWORKER_H
#define SPINWORKER_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>

class SpinWorker : public QObject
{
    Q_OBJECT
public:
    SpinWorker();
    ~SpinWorker();
    rclcpp::Node::SharedPtr             mNodePtr;

signals:

public slots:
    void process();
};

#endif // SPINWORKER_H