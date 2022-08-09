#include "logger.h"
#include <QDebug>
#include <QDateTime>

CLogger::CLogger(QString aFileName, QObject *parent)
:    QObject(parent)
{
    mFile.setFileName(aFileName);
    mFile.open(QIODevice::WriteOnly|QIODevice::Text);
    mTextStream.setDevice(&mFile);

    mCR = QString("\n");
}

CLogger::~CLogger()
{
    mTextStream.flush();
}

void CLogger::setTitle(const QString &string)
{
    mTextStream << string << mCR;
}

void CLogger::addLog(const QString &string)
{
    mTextStream << time() << string << mCR;
}

CLogger &CLogger::operator <<(const QString &string)
{
    mTextStream << time() << string << mCR;
    return *this;
}


CLogger &CLogger::operator <<(const char *string)
{
    mTextStream << time() << string << mCR;
    return *this;
}

QString CLogger::time()
{
    return QDateTime::currentDateTime().toString("yyyyMMdd:hhmmss.zzz") + QString(";");
}



//////////////////////////////////////////////////////
// NOT USED
//////////////////////////////////////////////////////

//CLogger &CLogger::operator <<(QChar c)
//{
//    mTextStream << c;
//    return *this;
//}

//CLogger &CLogger::operator <<(signed short i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(float f)
//{
//    mTextStream << f;
//    return *this;
//}

//CLogger &CLogger::operator <<(char c)
//{
//    mTextStream << c;
//    return *this;
//}

//CLogger &CLogger::operator <<(unsigned short i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(signed int i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(unsigned int i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(signed long i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(unsigned long i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(qlonglong i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(qulonglong i)
//{
//    mTextStream << i;
//    return *this;
//}

//CLogger &CLogger::operator <<(double f)
//{
//    mTextStream << f;
//    return *this;
//}

//CLogger &CLogger::operator <<(const QByteArray &array)
//{
//    mTextStream << array;
//    return *this;
//}
