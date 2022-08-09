#include "agentgraphicsitem.h"
#include "vehicle.h"

#include <QPainter>
#include <qmath.h>

CAgentGraphicsItem::CAgentGraphicsItem()
{

}

CAgentGraphicsItem::CAgentGraphicsItem(IVehicle *aAgent, QColor aColor)
{
    mAgent = aAgent;
    mColor = aColor;
}

QRectF CAgentGraphicsItem::boundingRect() const
{
    return QRectF(-500,-500, 1000, 1000);
}

void CAgentGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    const double PI = qAtan(1) * 4.0;
    const double d2r = PI/180.0;

    QPointF point;

	// TEST
	qreal offsety = 0;
	//if ( mAgent->id() == 21 ) {offsety = -8;}


	qreal posX = mAgent->data("posX").toReal();
	qreal posY = mAgent->data("posY").toReal() - offsety;
    qreal heading = mAgent->data("heading").toReal();
    qreal targetX = mAgent->data("targetX").toReal();
	qreal targetY = mAgent->data("targetY").toReal() - offsety;
    qreal distTarget = qSqrt(qPow(targetX - posX,2) + qPow(targetY - posY,2));



    // target ardrone structure
    if ( targetX != NONE_TARGET || targetY != NONE_TARGET ) {
        painter->setPen(mColor);
        painter->setPen(Qt::DashLine);
        painter->setBrush(mColor.lighter(150));

        // TODO : if ardrone is in target area, we should change solid pattern of brush
		QPointF targetPoint = meter2pixel(targetX, targetY);
        if ( distTarget < 0.10 ) {
            painter->drawEllipse(targetPoint, 10, 10);
        }
        else {
            painter->drawRect(targetPoint.x()-15, targetPoint.y()-15, 30,30);
        }
    }


	point = meter2pixel(posX,posY);
    painter->translate(point.x(), point.y());
    painter->rotate(-heading);

    // TEXT
    QStringList ipaddr = mAgent->info("ip").toString().split(".");

    painter->drawText(QPointF(-15,-40), QString("ID(%1:%2)")
                            .arg(mAgent->id())
                            .arg(ipaddr[3].toInt()-20));
    painter->drawText(QPointF(-15,-20), QString("Bat(%1\%)").arg(mAgent->data("battery").toString()));

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

void CAgentGraphicsItem::advance(int step)
{
    if (!step)
        return;

    update(-500,-500, 1000, 1000);
}

QPointF CAgentGraphicsItem::meter2pixel(qreal aX, qreal aY)
{
	const qreal pixelPer1m = 10.0;
	//return QPointF((aX - HALF_X)*pixelPer1m, (-aY + HALF_Y)*pixelPer1m);
	return QPointF((aX)*pixelPer1m, (-aY)*pixelPer1m);
}

qreal CAgentGraphicsItem::meter2pixel(qreal aX)
{
	const qreal pixelPer1m = 10.0;
	//return (aX - HALF_X)*pixelPer1m;
	return (aX)*pixelPer1m;
}

