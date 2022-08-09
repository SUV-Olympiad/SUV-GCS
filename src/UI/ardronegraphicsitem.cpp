#include "ardronegraphicsitem.h"
#include "vicondata.h"
#include <QPainter>
#include <qmath.h>

CARDroneGraphicsItem::CARDroneGraphicsItem()
{
    testRot = 0;

}

CARDroneGraphicsItem::CARDroneGraphicsItem(IVehicle *aAgent, QColor aColor)
{
    testRot = 0;
    mAgent = aAgent;
    mColor = aColor;
}

QRectF CARDroneGraphicsItem::boundingRect() const
{
//    qreal adjust = 20.5;
//    return QRectF(-50 - adjust, -50 - adjust, 100 + adjust, 100 + adjust);
    return QRectF(-500,-500, 1000, 1000);
}

void CARDroneGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    const double PI = qAtan(1) * 4.0;
    const double d2r = PI/180.0;

    QPointF point;


    // target ardrone structure
    if ( !mAgent->isTargetXNone() || !mAgent->isTargetYNone() || !mAgent->isTargetYNone() ) {
        painter->setPen(mColor);
        painter->setPen(Qt::DashLine);
        painter->setBrush(mColor.lighter(150));
        //painter->setBrush(Qt::DiagCrossPattern);


        // TODO : if ardrone is in target area, we should change solid pattern of brush
        QPointF targetPoint = meter2pixel(mAgent->targetX(), mAgent->targetY());
        if ( mAgent->distTarget() < 0.10 ) {
            painter->drawEllipse(targetPoint, 10, 10);
        }
        else {
            painter->drawRect(targetPoint.x()-15, targetPoint.y()-15, 30,30);
        }
    }


    point = meter2pixel(mAgent->viconData()->x(),mAgent->viconData()->y());
    painter->translate(point.x(), point.y());
    painter->rotate(-mAgent->viconData()->heading());

    // TEXT
    QStringList ipaddr = mAgent->ipAddr().split(".");

    painter->drawText(QPointF(-15,-40), QString("ID(%1:%2)")
                            .arg(mAgent->id())
                            .arg(ipaddr[3].toInt()-20));
    //painter->drawText(QPointF(-15,-20), QString("Bat(%1\%)").arg(mAgent->ardroneData()->battery()));

    // current ardrone strcture
    painter->setPen(Qt::SolidLine);
    const QPointF points[3] = {
         QPointF(  0.0,  15.0),
         QPointF(15*-qSin(120*d2r), 15.0*qCos(120*d2r)),
         QPointF(15*-qSin(-120*d2r), 15.0*qCos(-120*d2r))
    };
    painter->setBrush(mColor);
    painter->drawConvexPolygon(points, 3);
    painter->setBrush(Qt::black);
    painter->drawEllipse(QPointF(0.0, 15.0), 3, 3);
}

void CARDroneGraphicsItem::advance(int step)
{
    if (!step)
        return;

    testRot += 1.0;

    update(-500,-500, 1000, 1000);
}

QPointF CARDroneGraphicsItem::meter2pixel(qreal aX, qreal aY)
{
    const qreal HALF_X = 5.0;
    const qreal HALF_Y = 5.0;
    const qreal pixelPer1m = 50.0;
    return QPointF((aX - HALF_X)*pixelPer1m, (-aY + HALF_Y)*pixelPer1m);
}

qreal CARDroneGraphicsItem::meter2pixel(qreal aX)
{
    const qreal HALF_X = 5.0;
    const qreal pixelPer1m = 50.0;
    return (aX - HALF_X)*pixelPer1m;
}
