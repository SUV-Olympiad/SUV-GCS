#include "qhac_mapview.h"
#include <QtMath>
#include <QtDebug>
#include <QList>
#include <QDir>

#include <QPainterPath>

MapCache::MapCache()
{

}

MapCache::MapCache(int zoom, int x, int y)
{
    this->_x = x;
    this->_y = y;
    this->_z = zoom;
    this->_loaded = false;

    QString filepath = QString("./cache_imgs/%1/%2/%3.png").arg(_z).arg(_x).arg(_y);
    _fileInfo = QFileInfo(filepath);

    if ( _fileInfo.exists() == true ) {
        // load image file
        QImage img;
        img.load(_fileInfo.absoluteFilePath());

        // convert color to gray
        // img = img.convertToFormat(QImage::Format_Grayscale8);

        // convert QImage to QPixel
        _image.convertFromImage(img);

        this->_loaded = true;
    }


    _currentTile = nullptr;
    _nam = new QNetworkAccessManager();
    connect(_nam, SIGNAL (finished(QNetworkReply*)), SLOT (done(QNetworkReply*)));
}

bool MapCache::isMatch(int x, int y)
{
    if (this->_x == x && this->_y == y)
        return true;
    else
        return false;
}

void MapCache::setTile(MapTile *tile)
{
    if (this->_loaded)
    {
        tile->setPixmap(this->_image);
    }
    else
    {
        _currentTile = tile;
        _sx = tile->_posX;
        _sy = tile->_posY;
        QString url = QString("http://xdworld.vworld.kr:8080/2d/Satellite/service/%1/%2/%3.jpeg").arg(_z).arg(_x).arg(_y);
        QUrl    mapUrl;
        mapUrl.setUrl(url);
        QNetworkRequest request(mapUrl);
        QNetworkReply *rep = _nam->get(request);
//        connect(rep, &QNetworkReply::errorOccurred, this, &MapCache::errorOccurred);
//        qDebug() << url;
    }

}

void MapCache::clearTile()
{
    _currentTile = nullptr;
}

void MapCache::done(QNetworkReply* pReply)
{
    QByteArray  mapImageData;

    mapImageData = pReply->readAll();
    pReply->deleteLater();

    // convert binary to QImage
    QImage img = QImage::fromData(mapImageData);

    // save map image to file
    if ( _fileInfo.exists() == false ) {
        QDir().mkpath(_fileInfo.absolutePath());
        img.save(_fileInfo.absoluteFilePath());
    }

    // convert color to gray
    // img = img.convertToFormat(QImage::Format_Grayscale8);

    // convert QImage to QPixel
    _image.convertFromImage(img);

    _loaded = true;
    if (_currentTile) {
        if (_currentTile->_posX == _sx && _currentTile->_posY == _sy)        {
            _currentTile->setPixmap(_image);
        }
        else {
            //qDebug() << "mismatch :" << currentTile->posX << ":" << currentTile->posY;
        }
    }
    else    {
        qDebug() << "??" << _z << ":" << _x << ":" << _y;;
    }
    _nam->deleteLater();
}

void MapCache::errorOccurred(QNetworkReply::NetworkError code)
{
    qDebug() << _z << ":" << _x << ":" << _y;
    qDebug() << code;
}

MapCacheManager::MapCacheManager()
{

}

MapCache* MapCacheManager::getMapCache(int zoom, int x, int y)
{
    QList<MapCache*> mcList;
    MapCache *mc;
    mcList = _mcl[zoom];

    // search
    for (int i = 0; i < mcList.size(); i++)
    {
        if (mcList[i]->isMatch(x, y)) return mcList[i];
    }

    mc = new MapCache(zoom, x, y);
    _mcl[zoom].append(mc);

    return mc;
}


MapTile::MapTile(QWidget *parent) : QLabel (parent)
{
    //this->setStyleSheet("border: 1px solid blue;");
}

void MapTile::mousePressEvent(QMouseEvent *ev)
{
    if (ev->button() == Qt::LeftButton) {
        qDebug() << "sx:" << this->_posX << " sy:" << this->_posY;
    }

    QFrame::mousePressEvent(ev);
}

bool MapTile::movePos(QPoint pos)
{
    QPoint dPos = _startPos + pos;
    int size = this->width();

    if (dPos.x() < -(size+(size>>1))) {
        _posX += _maxW;
        dPos.setX(dPos.x() + (size*_maxW));
    }
    if (dPos.y() < -(size+(size>>1))) {
        _posY += _maxH;
        dPos.setY(dPos.y() + (size*_maxH));
    }
    if (dPos.x() > ((_maxW - 2) * size) + (size>>1)) {
        _posX -= _maxW;
        dPos.setX(dPos.x() - (size*_maxW));
    }
    if (dPos.y() > ((_maxW - 2) * size) + (size>>1)) {
        _posY -= _maxH;
        dPos.setY(dPos.y() - (size*_maxH));
    }

    move(dPos);

    if (dPos != _startPos + pos) {
        _startPos = dPos - pos;
        return true;
    }
    else {
        return false;
    }
}

void MapTile::setStartPoint(void)
{
    _startPos = pos();
}

qhac_mapview::qhac_mapview(QWidget *parent) : QFrame (parent)
{
    _offset_tile_pos = QPointF(0.0,0.0);
}

void qhac_mapview::init(CManager* aManager, int w, int h)
{
    if (_initialized) return;
    _initialized = true;

    if (w > 10 || h > 10 || w < 1 || h < 1) {
        qWarning("init failed: w, h");
        return;
    }

    _map_w = w + 2;
    _map_h = h + 2;

    _mapTile = new MapTile*[_map_w*_map_h];

    if (_mapTile == NULL) {
        qWarning("init failed: malloc");
        return;
    }

    for (int i = 0 ; i < _map_w * _map_h ; i++) {
        int x, y;
        MapTile *tile = new MapTile(this);

        tile->_maxW = _map_w;
        tile->_maxH = _map_h;

        x = ((i % _map_w) - 1) * DEF_TILE_SIZE;
        y = ((i / _map_w) - 1) * DEF_TILE_SIZE;
        tile->setGeometry(x, y, DEF_TILE_SIZE, DEF_TILE_SIZE);
        tile->setScaledContents(true);
        tile->setText("Loading...");
        tile->_deferred_resize = false;
        tile->show();
        _mapTile[i] = tile;
    }

    _tile_size = DEF_TILE_SIZE;

    // setup object view
    _objectView = new ObjectView(this);
    _objectView->show();


    _objectView->updateManager(aManager);

    // setup image view
//    _imageView = new ImageView(this);
//    _imageView->show();

}

void qhac_mapview::loadTile(MapTile *mt, int zoom, int sx, int sy)
{
    if (zoom < MIN_ZOOM || zoom > MAX_ZOOM) return;

    mt->_posX = sx;
    mt->_posY = sy;
    mt->_curZ = zoom;

    MapCache *mc = _mcm.getMapCache(zoom, sx, sy);
    mc->setTile(mt);
}

void qhac_mapview::reset(void)
{
    for (int i = 0; i < _map_w * _map_h; i++) {
        int x, y;
        MapTile *tile = _mapTile[i];

        tile->_maxW = _map_w;
        tile->_maxH = _map_h;

        x = ((i % _map_w) - 1) * DEF_TILE_SIZE;
        y = ((i / _map_w) - 1) * DEF_TILE_SIZE;
        tile->setGeometry(x, y, DEF_TILE_SIZE, DEF_TILE_SIZE);
        tile->setScaledContents(true);
        tile->setText("Loading...");
    }
}

void qhac_mapview::load(int zoom, int x, int y)
{    
    if (zoom >= MIN_ZOOM && zoom <= MAX_ZOOM) {
        _map_zoom = zoom;
    } else {
        qWarning("zoom value is invalid");
        return;
    }

    int sx = x - 1;
    int sy = y - 1;
    for (int i = 0; i < _map_w * _map_h; i++) {
        _mapTile[i]->setStartPoint();
        loadTile(_mapTile[i], _map_zoom, sx + (i % _map_w), sy + (i / _map_w));
    }
}

void qhac_mapview::setZoom(int zoom)
{
    _map_zoom = zoom;
}

QPoint qhac_mapview::setTileSize(int size, QPoint pos, bool reload)
{
    QPoint movePoint = QPoint(0,0);
    int dx, dy, mx, my, x, y, ds, mapx, mapy, x1, x2, y1, y2, px, py;
    bool oddx = false, oddy = false;
    ds = size - _tile_size;

    px = pos.x();
    py = pos.y();
    dx = px - _mapTile[0]->x();
    dy = py - _mapTile[0]->y();

    if ((dx % _tile_size) == 0) {
        dx++;
        px++;
    }
    if ((dy % _tile_size) == 0) {
        dy++;
        py++;
    }

    mx = dx / _tile_size;
    my = dy / _tile_size;
    mx = dx < 0 ? mx - 1 : mx;
    my = dy < 0 ? my - 1 : my;

    if ((_mapTile[0]->_posX - mx) & 1) {
        oddx = true;
    }

    if ((_mapTile[0]->_posY - my) & 1) {
        oddy = true;
    }

    if (dx > 0) {
        x1 = dx % _tile_size;
    }
    else {
        x2 = ((-dx) % _tile_size);
        x1 = _tile_size - x2;
    }

    if (dy > 0) {
        y1 = dy % _tile_size;
    }
    else {
        y2 = ((-dy) % _tile_size);
        y1 = _tile_size - y2;
    }

    x1 = ds * x1 / _tile_size;
    x2 = ds - x1;
    y1 = ds * y1 / _tile_size;
    y2 = ds - y1;

    if (reload) {
        if (size == MIN_TILE_SIZE) {
            movePoint.setX(x1);
            movePoint.setY(y1);
        }
        else {
            movePoint.setX(oddx ? (-x2) : x1);
            movePoint.setY(oddy ? (-y2) : y1);
        }
    }

    for (int i = 0; i < _map_w * _map_h; i++) {
        dx = px - _mapTile[i]->x();
        dy = py - _mapTile[i]->y();
        mx = dx / _tile_size;
        my = dy / _tile_size;
        x = dx < 0 ? ((mx) * ds) - x2 : ((mx) * ds) + x1;
        y = dy < 0 ? ((my) * ds) - y2 : ((my) * ds) + y1;

        _mapTile[i]->setStartPoint();

        QPoint pos2 = QPoint(-x, -y);

        if (reload) {
            mx = dx < 0 ? mx - 1 : mx;
            my = dy < 0 ? my - 1 : my;

            if (size == MIN_TILE_SIZE) {
                // zoom in
                mapx = (_mapTile[i]->_posX << 1) + mx;
                mapy = (_mapTile[i]->_posY << 1) + my;

                loadTile(_mapTile[i], _map_zoom, mapx, mapy);

                pos2 += movePoint;
            }
            else  {
                // zoom out
                mapx = (_mapTile[i]->_posX - mx) >> 1;
                mapy = (_mapTile[i]->_posY - my) >> 1;

                loadTile(_mapTile[i], _map_zoom, mapx, mapy);

                pos2 += movePoint;
            }
        }

        _mapTile[i]->resize(size,size);
        if (_mapTile[i]->movePos(pos2)) {
            loadTile(_mapTile[i], _mapTile[i]->_curZ, _mapTile[i]->_posX, _mapTile[i]->_posY);
        }

        if (dx == 0 || dy == 0) {
            qDebug() << "oops!" << ":" << dx << ":" << dy << ":" << oddx << ":" << oddy << ":" << size << ":" << reload << ":" << pos2.x() << ":" << pos2.y();
        }        
    }
    _tile_size = size;

    // validation
//    int xx = (_mapTile[0]->x()+(size*2)) % size;
//    int yy = (_mapTile[0]->y()+(size*2)) % size;
//    for (int i = 1; i < _map_w * _map_h; i++) {
//        if ((xx != ((_mapTile[i]->x()+(size*2)) % size)) ||
//            (yy != ((_mapTile[i]->y()+(size*2)) % size))) {
//            qDebug() << "OMG!:" << ds << ":" << size << ":" << x1 << ":" << x2 << ":" << y1 << ":" << y2 << "xx" << xx << "yy" << yy;
//            qDebug() << _mapTile[i]->x() << ":" << _mapTile[i]->y() << ":" << reload;
//        }

//    }

    movePoint.setX(0);
    movePoint.setY(0);
    if (oddx) movePoint.setX(size/2);
    if (oddy) movePoint.setY(size/2);

    return movePoint;
}

void qhac_mapview::moveTiles(QPoint pos)
{    
    for (int i = 0; i < _map_w * _map_h; i++) {
        if (_mapTile[i]->movePos(pos - _curTilePos)) {
            loadTile(_mapTile[i], _mapTile[i]->_curZ, _mapTile[i]->_posX, _mapTile[i]->_posY);
        }
    }
}

void qhac_mapview::setTilePoint(QPoint pos)
{
    for (int i = 0; i < _map_w * _map_h; i++) {
        _mapTile[i]->setStartPoint();
    }
}

void qhac_mapview::moveByGPS(double lat, double lon, int zoom)
{
    this->reset();

    _map_zoom = zoom;

    _origin_llh = QPointF(lat, lon);

    QPointF pos = LLH2TilePos(QPointF(lat,lon));
    qDebug("original pos : %.9f %.9f", pos.x(), pos.y());
    _origin_tile_pos = pos;

    // load image and fix the position    
    load(zoom, qFloor(pos.x()/_tile_size), qFloor(pos.y()/_tile_size));
    moveTiles(-QPoint(pos.toPoint().x()%_tile_size, pos.toPoint().y()%_tile_size));
}

void qhac_mapview::updateDrone(int id, qreal lat, qreal lon, float heading)
{
    _objectView->updateDrone(id, QPointF(lat, lon), heading);
}

void qhac_mapview::updateColor(QMap<int, QColor> colorList)
{
    _objectView->updateColor(colorList);
}

void qhac_mapview::selectVehicle(int selectId)
{
    _objectView->selectVehicle(selectId);
}

void qhac_mapview::updateRoad(QMap<int, QString> roadList)
{
    _objectView->updateRoad(roadList);
}

void qhac_mapview::updateImage(QRectF region, QImage image, qreal rot, int num)
{
    _objectView->updateImage(region, image, rot, num);
}

QPointF qhac_mapview::LLH2TilePos(QPointF llh)
{
    const qreal scale = 1 << _map_zoom;
    qreal lat = llh.x();
    qreal lon = llh.y();

    qreal siny = qSin((lat * M_PI) / 180.0);
    // Truncating to 0.9999 effectively limits latitude to 89.189. This is
    // about a third of a tile past the edge of the world tile.
    siny = qMin(qMax(siny, -0.9999), 0.9999);

    qreal x = (0.5 + lon / 360.0) * scale * _tile_size;
    qreal y = (0.5 - qLn((1.0 + siny) / (1.0 - siny)) / (4.0 * M_PI)) * scale * _tile_size;

    return QPointF(x,y);
}

QPointF qhac_mapview::Tile2LLHPos(QPointF pos)
{
    const qreal scale = 1 << _map_zoom;

    qreal lon = ((pos.x() / (scale * _tile_size)) - 0.5)* 360.0;
    qreal k = (0.5 - (pos.y() / (scale * _tile_size))) * 4.0 * M_PI;
    qreal lat = qAsin((qExp(k) - 1) / (qExp(k) + 1)) * (180.0 /M_PI);

    return QPointF(lat, lon);
}

void qhac_mapview::mouseMoveEvent(QMouseEvent *ev) {
    if (ev->buttons() & Qt::LeftButton) {

        moveTiles(ev->pos());        
        _offset_tile_pos = ev->pos() - _curTilePos;
    }

    QFrame::mouseMoveEvent(ev);
    emit Mouse_Pos();
}

void qhac_mapview::mousePressEvent(QMouseEvent *ev)
{
    if (ev->button() == Qt::LeftButton) {
        this->_curTilePos = ev->pos();
        setTilePoint(ev->pos());
    }

    QFrame::mousePressEvent(ev);
    emit Mouse_Pressed();    
}

void qhac_mapview::mouseReleaseEvent(QMouseEvent *ev)
{    
    _origin_tile_pos -= _offset_tile_pos;
    _origin_llh = Tile2LLHPos(_origin_tile_pos);
    _offset_tile_pos = QPoint(0.0,0.0);

    QFrame::mouseReleaseEvent(ev);
    emit Mouse_Released();    
}

void qhac_mapview::wheelEvent(QWheelEvent *ev)
{
    int size;
    QPoint point = ev->angleDelta() / 8, mPoint;
    int dy = point.y();
    bool reload = false;

    if (abs(dy) >= 10) {
        // get event tile
        int _dx;
        int _dy;
        MapTile *event_tile;
        QPoint event_tile_pos;

        for (int i = 0; i < _map_w * _map_h; i++) {
            _dx = ev->pos().x() - _mapTile[i]->x();
            _dy = ev->pos().y() - _mapTile[i]->y();
            if (_dx >= 0 && _dx < _tile_size && _dy >= 0 && _dy < _tile_size) {
                event_tile = _mapTile[i];
                event_tile_pos = QPoint(_mapTile[i]->x(),_mapTile[i]->y());
                break;
            }
        }

        QPointF llh_ev = Tile2LLHPos(_origin_tile_pos + event_tile_pos);

        size = _tile_size + dy;
        if (size > MAX_TILE_SIZE) {
            // zoom in
            if (_map_zoom == MAX_ZOOM) {
                size = MAX_TILE_SIZE;
            }
            else {
                size = MIN_TILE_SIZE;
                _map_zoom++;
                reload = true;
            }
        }
        else if (size < MIN_TILE_SIZE) {
            // zoom out
            if (_map_zoom == MIN_ZOOM) {
                size = MIN_TILE_SIZE;
            }
            else {
                size = MAX_TILE_SIZE;
                _map_zoom--;
                reload = true;
            }
        }
        size &= ~(0x1);

        mPoint = setTileSize(size, ev->pos(), reload);

        // update origin tile pos
        event_tile_pos = QPoint(event_tile->x(), event_tile->y());
        if (reload && size == MAX_TILE_SIZE) {
            event_tile_pos += mPoint;
        }
        _origin_tile_pos = LLH2TilePos(llh_ev) - event_tile_pos;
        _origin_llh = Tile2LLHPos(_origin_tile_pos);

    }

    QFrame::wheelEvent(ev);
}

void qhac_mapview::leaveEvent(QEvent *ev)
{
    emit Mouse_Left();

    QFrame::leaveEvent(ev);
}

void qhac_mapview::resizeEvent(QResizeEvent* ev)
{
    emit Mapview_Resized(ev);
    QFrame::resizeEvent(ev);    
}

ObjectView::ObjectView(qhac_mapview *parent) : QLabel (parent)

{
    _mapview = parent;

    // connect
    connect(parent, SIGNAL (Mapview_Resized(QResizeEvent*)), SLOT (resize_event(QResizeEvent*)));
}

void ObjectView::updateManager(CManager* aManager){
    _selectVehicleId = -1;
    _manager = aManager;
}

void ObjectView::updateColor(QMap<int, QColor> colorList)
{
    _colorList = colorList;
}

void ObjectView::selectVehicle(int selectId)
{
    _selectVehicleId = selectId;
}


void ObjectView::updateRoad(QMap<int, QString> roadList)
{
    _roadList = roadList;
}

void ObjectView::updateDrone(int id, QPointF llh, float heading)
{
    if ( _droneList.contains(id) ) {
        _droneList[id].update(llh, heading);
    }
    else {
        _droneList[id] = DroneObject(id, llh, heading);
    }

    this->update();
}

void ObjectView::updateImage(QRectF region, QImage image, qreal rot, int num)
{
    _img_region[num] = region;
    _img_rot[num] = rot;
    _image[num] = image;

    this->update();
}

void ObjectView::paintEvent(QPaintEvent *event)
{

    QPointF origin_pos = _mapview->originTilePos() - _mapview->offsetTilePos();

    QPainter paint(this);

    // draw drone image #1
    paint.setOpacity(0.8);
    QPointF img_pos = _mapview->LLH2TilePos(QPointF(_img_region[0].x(), _img_region[0].y()));
    QPointF img_pos2 = _mapview->LLH2TilePos(QPointF(_img_region[0].x() + _img_region[0].width(), _img_region[0].y() + _img_region[0].height()));

    QPointF pos = img_pos - origin_pos;
    QPointF size = img_pos2 - img_pos;
    QRect region = QRect(pos.x(), pos.y(),  qFabs(size.x()), qFabs(size.y()));

//    paint.translate((double)(region.width()/2),(double)(region.height()/2));
//    paint.rotate(-5);
//    paint.translate((double)(-region.width()/2),(double)(-region.height()/2));
    paint.drawPixmap(region, QPixmap::fromImage(_image[0]));


    // draw drone image #2
    img_pos = _mapview->LLH2TilePos(QPointF(_img_region[1].x(), _img_region[1].y()));
    img_pos2 = _mapview->LLH2TilePos(QPointF(_img_region[1].x() + _img_region[1].width(), _img_region[1].y() + _img_region[1].height()));

    pos = img_pos - origin_pos;
    size = img_pos2 - img_pos;
    region = QRect(pos.x(), pos.y(),  size.x(), size.y());
    QPointF center = QPointF(pos.x()+region.width()/2, pos.y()+region.height()/2);
    paint.translate(center);
    paint.rotate(-5);
    paint.translate(-center);
    paint.drawPixmap(region, QPixmap::fromImage(_image[1]));
    paint.translate(center);
    paint.rotate(5);
    paint.translate(-center);

    // draw drone image #3
    img_pos = _mapview->LLH2TilePos(QPointF(_img_region[2].x(), _img_region[2].y()));
    img_pos2 = _mapview->LLH2TilePos(QPointF(_img_region[2].x() + _img_region[2].width(), _img_region[2].y() + _img_region[2].height()));

    pos = img_pos - origin_pos;
    size = img_pos2 - img_pos;
    region = QRect(pos.x(), pos.y(),  size.x(), size.y());
    paint.drawPixmap(region, QPixmap::fromImage(_image[2]));


    // draw drone image #4
    img_pos = _mapview->LLH2TilePos(QPointF(_img_region[3].x(), _img_region[3].y()));
    img_pos2 = _mapview->LLH2TilePos(QPointF(_img_region[3].x() + _img_region[3].width(), _img_region[3].y() + _img_region[3].height()));

    pos = img_pos - origin_pos;
    size = img_pos2 - img_pos;
    region = QRect(pos.x(), pos.y(),  size.x(), size.y());
    paint.drawPixmap(region, QPixmap::fromImage(_image[3]));
    paint.setOpacity(1.0);


    if(_selectVehicleId != -1){
        _droneList[INT_MAX] = _droneList[_selectVehicleId];
    }
    
    foreach (DroneObject drone, _droneList) {
            
        paint.setOpacity(0.5);
        if(_selectVehicleId == drone.id()){
            paint.setOpacity(1.0);
        }

        QPen pen(_colorList[drone.id()],10);
        paint.setPen(pen);

        paint.setBrush(_colorList[drone.id()]);

        IVehicle* agent =  _manager->agent(drone.id());
        QList<QVariant> list = agent->data("MISSION").toList();
        agent->data("OFFLINE");

        QPointF start, end;
        // Draw waypoint
        for (int i = 1; i < list.size(); ++i) {
            CROSData::MissionItem *item = list[i].value<CROSData::MissionItem*>();
            float lat = item->lat;
            float lon = item->lon;
            if (i == 0) {
                start = QPointF(lat, lon);
            } else if (i == 1) {
                end = QPointF(lat, lon);
            }
            QPointF startPosTile = _mapview->LLH2TilePos(QPointF(lat,lon)) - origin_pos;
            paint.drawEllipse(QPointF(startPosTile.x(),startPosTile.y()), 5, 5);
        }

        // Draw Line
        for (int i = 0; i < list.size() - 1; ++i) {
            CROSData::MissionItem *item = list[i].value<CROSData::MissionItem*>();
            CROSData::MissionItem *item2 = list[i+1].value<CROSData::MissionItem*>();
            float start_lat = item->lat;
            float start_lon = item->lon;
            float end_lat = item2->lat;
            float end_lon = item2->lon;

            QPointF startPosTile = _mapview->LLH2TilePos(QPointF(start_lat,start_lon)) - origin_pos;
            QPointF endPosTile = _mapview->LLH2TilePos(QPointF(end_lat,end_lon)) - origin_pos;
            paint.drawLine(startPosTile.x(), startPosTile.y(), endPosTile.x(), endPosTile.y());
        }

        QPointF drone_pos = _mapview->LLH2TilePos(drone.llh());
        QPointF pos = drone_pos - origin_pos;
        float heading = drone.heading();
        // qDebug("drone pos[%d] : %.9f %.9f (%.9f, %.9f)", drone.id(), pos.x(), pos.y(), drone.llh().x(), drone.llh().y());

        

        paint.save();
        paint.translate(pos.x(),pos.y());
        paint.rotate(qRadiansToDegrees(heading) + 90);
        QPixmap pix(_manager->vehicleImage(_manager->vehicleType(drone.id())));
        paint.drawPixmap(-25, -25, 50, 50, pix);
        paint.restore();

        QFont font = paint.font();
        font.setPixelSize(15);
        font.bold();
        paint.setFont(font);

        int color = _colorList[drone.id()].red() + _colorList[drone.id()].green() + _colorList[drone.id()].blue() / 3;
        
        if(color >= 128){
            paint.setPen(Qt::black);
        }else{
            paint.setPen(Qt::white);
        }

        paint.fillRect(QRect(pos.x() - 5, pos.y() + 25, 50, 30),QBrush(_colorList[drone.id()]));

        QPointF text_pos = QPointF(pos.x(), pos.y() + 45);
        paint.drawText(text_pos, QString("[%1] %2").arg(_manager->groupId(drone.id())).arg(drone.id()));


        // QRectF rect = QRectF(pos.x(), pos.y(), 50, 50);
        // QPainterPath path;

        // QPointF p1 = QPointF(rect.left(), rect.top());
        // QPointF p2 = QPointF(rect.left() - (rect.width() / 2), rect.bottom());
        // QPointF p3 = QPointF(rect.right() - (rect.width() / 2), rect.bottom());

        // path.moveTo(rotate(p1, pos, heading));
        // path.lineTo(rotate(p2, pos, heading));
        // path.lineTo(rotate(p3, pos, heading));
        // path.lineTo(rotate(p4, pos, heading));
        
        // paint.fillPath(path, QBrush(_colorList[drone.id()]));
        // paint.setBrush(QBrush(Qt::red));
        // paint.drawEllipse(pos, 5, 5);
    }




    // draw interesting region
    paint.setPen(QColor(Qt::red));
    paint.setBrush(QBrush(Qt::blue));
    for ( int i = 0 ; i < _regionLLH.size() ; i++ ) {
        QPointF cp = _mapview->LLH2TilePos(_regionLLH[i]) - origin_pos;

        if ( i == 0 ) {
            paint.drawRect(QRectF(cp.x()-DOT_SIZE/2, cp.y()-DOT_SIZE/2, DOT_SIZE,DOT_SIZE));
        }
        else {
            QPointF p0 = _mapview->LLH2TilePos(_regionLLH[0]) - origin_pos;
            QPointF pp = _mapview->LLH2TilePos(_regionLLH[i-1]) - origin_pos;
            int dist = qSqrt(qPow(p0.x()-cp.x(),2) + qPow(p0.y()-cp.y(),2));
            if ( dist > DOT_SIZE ) {
                paint.drawLine(pp.x(), pp.y(), cp.x(), cp.y());
                paint.drawRect(QRectF(cp.x()-DOT_SIZE/2, cp.y()-DOT_SIZE/2, DOT_SIZE,DOT_SIZE));
            }
            else {
                paint.drawLine(pp.x(), pp.y(), p0.x(), p0.y());
            }
        }
    }


    paint.end();

    QFrame::paintEvent(event);

}

void ObjectView::mouseReleaseEvent(QMouseEvent *ev)
{
    QPointF originPos = _mapview->originTilePos() - _mapview->offsetTilePos();

    if(Qt::ControlModifier == QApplication::keyboardModifiers()) {        
        if ( _region.size() > 1 ) {
            QPointF p0 = _region[0];
            QPointF cp = _region.last();
            int dist = qSqrt(qPow(p0.x()-cp.x(),2) + qPow(p0.y()-cp.y(),2));
            if ( dist < DOT_SIZE ) {                
                _region.clear();
                _regionLLH.clear();
            }
        }

        QPointF pt = originPos + ev->pos();
        _region.append(pt);

        QPointF llh = _mapview->Tile2LLHPos(pt);
        _regionLLH.append(llh);
    }
    this->update();

    QFrame::mouseReleaseEvent(ev);
}

void ObjectView::wheelEvent(QWheelEvent *ev)
{
    QFrame::wheelEvent(ev);
    this->update();
}

QPointF ObjectView::rotate(QPointF pos, QPointF base, qreal rot)
{
    qreal theta = (3.14f / 180.0f) * qRadiansToDegrees(rot);
    qreal x = (pos.x() - base.x())*qCos(theta) - (pos.y()-base.y())*qSin(theta) + base.x();
    qreal y = (pos.x() - base.x())*qSin(theta) + (pos.y()-base.y())*qCos(theta) + base.y();

    return QPointF(x,y);
}

void ObjectView::resize_event(QResizeEvent* ev)
{
//    qDebug("mapview resize %d,%d", ev->size().width(), ev->size().height());
    this->setGeometry(0,0,ev->size().width(), ev->size().height());
}

DroneObject::DroneObject() :
    _id(0), _heading(0)
{

}

DroneObject::DroneObject(int id, QPointF llh, float heading) :
    _id(id), _llh(llh), _heading(heading)
{

}

void DroneObject::update(QPointF llh, float heading)
{
    _llh = llh;
    _heading = heading;
}
