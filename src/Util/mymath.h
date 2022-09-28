#ifndef MYMATH_H
#define MYMATH_H

#include <math.h>
// Jang added 2015. 11. 13.
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <QtGlobal>
#include <QtCore/qmath.h>
#include <cmath>
#include <QVector2D>
#include <QLineF>

//#define _PI_        (3.14159265358979)
#define R2D         (180.0/M_PI)
#define D2R         (M_PI/180.0)


float doublePI(float value);
// Jang added 2015. 11. 13.
Eigen::Vector3f rotVec(Eigen::Vector3f a, Eigen::Vector3f b);
float angle(Eigen::Vector3f a, Eigen::Vector3f b);
Eigen::Matrix3f rodrigues(Eigen::Vector3f v);
double yaw3D(Eigen::Matrix3f R);
int sign(float x);
float trim(float x);
Eigen::Matrix3f rotateX(float roll);
Eigen::Matrix3f rotateY(float pitch);
Eigen::Matrix3f rotateZ(float yaw);


double ccw(QVector2D a, QVector2D b);
double ccw(QVector2D p, QVector2D a, QVector2D b);
bool sementIntersects(QVector2D a, QVector2D b, QVector2D c, QVector2D d);
bool sementIntersects(QLineF line1, QLineF line2);

#endif // MYMATH_H
