#include "spinworker.h"
#include "sleeper.h"
#include <QDebug>

SpinWorker::SpinWorker()
{

}


SpinWorker::~SpinWorker()
{

}

void SpinWorker::process()
{
    rclcpp::spin(mNodePtr);
}
