#ifndef ARDRONEGRAPHICSITEM_H
#define ARDRONEGRAPHICSITEM_H

#include <QGraphicsItem>
#include <QPointF>
#include <QColor>
#include "vehicle.h"

class CARDroneGraphicsItem : public QGraphicsItem
{
public:
    explicit CARDroneGraphicsItem();
    explicit CARDroneGraphicsItem(IVehicle* aAgent, QColor aColor);
    
public:
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    void advance(int step);

private:
    QPointF meter2pixel(qreal aX, qreal aY);
    qreal meter2pixel(qreal aX);

private:
    qreal testRot;

    IVehicle*                 mAgent;
    QColor                  mColor;
};

#endif // ARDRONEGRAPHICSITEM_H
