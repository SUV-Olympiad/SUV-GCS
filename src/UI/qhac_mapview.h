#ifndef QHAC_MAPVIEW_H
#define QHAC_MAPVIEW_H

#include <QWidget>
#include <QFrame>
#include <QLabel>
#include <QList>
#include <QMap>
#include <QPixmap>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QApplication>
#include <QByteArray>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QPainterPath>
#include <QFileInfo>

class qhac_mapview;

class DroneObject
{
public:
    explicit DroneObject();
    explicit DroneObject(int id, QPointF llh, float heading=0);

public:
    int id() {return _id;}
    void updateHeading(float heading) {_heading = heading;}
    void update(QPointF llh, float heading=0);
    QPointF llh() {return _llh;}
    float heading() {return _heading;}

private:
    int             _id;
    float           _heading;
    QPointF         _llh;

};

class ObjectView : public QLabel
{
    Q_OBJECT

public:
    const int                   DOT_SIZE = 10;
public:
    explicit ObjectView(qhac_mapview *parent);
    void updateDrone(int id, QPointF llh, float heading=0);
    void updateColor(QMap<int, QColor>);
    void updateImage(QRectF region, QImage image, qreal rot=0.0, int num=0);

protected:
    void paintEvent(QPaintEvent* event);
    void mouseReleaseEvent(QMouseEvent *ev);
    void wheelEvent(QWheelEvent *ev);

private:
    QPointF rotate(QPointF pos, QPointF base, qreal rot);

private:
    qhac_mapview*               _mapview;
    QPixmap                     _pixmap;
    QList<QPointF>              _region;
    QList<QPointF>              _regionLLH;
    QMap<int,DroneObject>       _droneList;
    QMap<int, QColor>           _colorList;

    QRectF                      _img_region[4];    // region of drone image
    qreal                       _img_rot[4];       // rotation of drone image
    QImage                      _image[4];         // drone image


//    QRectF                      _img_region2;    // region of drone image
//    qreal                       _img_rot2;       // rotation of drone image
//    QImage                      _image2;         // drone image

//    QRectF                      _img_region3;    // region of drone image
//    qreal                       _img_rot3;       // rotation of drone image
//    QImage                      _image3;         // drone image

//    QRectF                      _img_region4;    // region of drone image
//    qreal                       _img_rot4;       // rotation of drone image
//    QImage                      _image4;         // drone image
private slots:
    void resize_event(QResizeEvent* ev);
};

class MapTile : public QLabel
{
    Q_OBJECT
public:
    explicit MapTile(QWidget *parent = 0);
    bool movePos(QPoint pos);
    void setStartPoint(void);
    void mousePressEvent(QMouseEvent *ev);

    int                         _maxW;
    int                         _maxH;
    bool                        _deferred_resize;
    int                         _deferred_x, _deferred_y, _deferred_size;
    int                         _posX;
    int                         _posY;
    int                         _curZ;
    int                         _mxZoomIn, _myZoomIn;
    int                         _mxZoomOut, _myZoomOut;

private:
    QPoint                      _startPos;
};

class MapCache : public QObject
{
    Q_OBJECT
public:
    MapCache();
    MapCache(int zoom, int x, int y);
    bool isMatch(int x, int y);
    void setTile(MapTile *mapTile);
    void clearTile();

private:
    int                         _x;
    int                         _y;
    int                         _z;
    int                         _sx, _sy;
    QPixmap                     _image;
    MapTile*                    _currentTile;
    bool                        _loaded;
    QNetworkAccessManager*      _nam;
    QFileInfo                   _fileInfo;

private slots:
    void done(QNetworkReply *pReply);
    void errorOccurred(QNetworkReply::NetworkError code);

};

class MapCacheManager : public QObject
{
    Q_OBJECT
public:
    MapCacheManager();
    MapCache* getMapCache(int zoom, int x, int y);

private:
    QList<MapCache*>            _mcl[20];
};

class qhac_mapview : public QFrame
{
    Q_OBJECT

public:
    explicit qhac_mapview(QWidget *parent = 0);
    void init(int w, int h);
    void moveByGPS(double lat, double lon, int zoom);
    void updateDrone(int id, qreal lat, qreal lon, float heading=0);
    void updateColor(QMap<int, QColor>);
    void updateImage(QRectF region, QImage image, qreal rot=0, int num=0);


    QPointF originTilePos() {return _origin_tile_pos;}
    QPointF offsetTilePos() {return _offset_tile_pos;}
    int zoom() {return _map_zoom;}
    int tile_size() {return _tile_size;}

    QPointF LLH2TilePos(QPointF llh);
    QPointF Tile2LLHPos(QPointF pos);

protected:
    void mouseMoveEvent(QMouseEvent *ev);
	void mousePressEvent(QMouseEvent *ev);
	void mouseReleaseEvent(QMouseEvent *ev);
    void wheelEvent(QWheelEvent *ev);
	void leaveEvent(QEvent *);
    void resizeEvent(QResizeEvent* ev);

    void load(int zoom, int x, int y);
    void setZoom(int zoom);
    void moveTiles(QPoint pos);
    void setTilePoint(QPoint pos);
    QPoint setTileSize(int size, QPoint pos, bool reload);
    void setPoint(double latitude, double longitude);
    void reset(void);

    MapTile**                   _mapTile;
    QPoint                      _curTilePos;

    QPointF                      _origin_llh;               // Not used until now
    QPointF                      _origin_tile_pos;
    QPointF                      _offset_tile_pos;
    int                         _map_zoom;
    int                         _tile_size;

private:
    void loadTile(MapTile *mt, int zoom, int sx, int sy);

private:
    const int                   MIN_ZOOM = 6;
    const int                   MAX_ZOOM = 19;
    const int                   MIN_TILE_SIZE = 256;
    const int                   DEF_TILE_SIZE = 384;
    const int                   MAX_TILE_SIZE = 512;
    int                         _map_h, _map_w;

    bool                        _initialized = false;

    MapCacheManager             _mcm;

private:
    ObjectView*                 _objectView;

signals:
    void Mouse_Pressed();
	void Mouse_Released();
	void Mouse_Pos();
	void Mouse_Left();
    void Mapview_Resized(QResizeEvent* event);
    void addStatusItem(const QString &label);

private slots:


};

#endif // QHAC_MAPVIEW_H
