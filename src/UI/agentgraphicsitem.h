#ifndef CAGENTGRAPHICSITEM_H
#define CAGENTGRAPHICSITEM_H

#include <QGraphicsItem>
#include <QPointF>
#include <QColor>

class IVehicle;

class CAgentGraphicsItem : public QGraphicsItem
{
public:
    explicit CAgentGraphicsItem();
    explicit CAgentGraphicsItem(IVehicle* aAgent, QColor aColor);

public:
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    void advance(int step);

private:
    QPointF meter2pixel(qreal aX, qreal aY);
    qreal meter2pixel(qreal aX);

private:
    IVehicle*                 mAgent;
    QColor                  mColor;

};

#endif // CAGENTGRAPHICSITEM_H
