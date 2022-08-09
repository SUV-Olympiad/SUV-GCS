#ifndef CLOGGER_H
#define CLOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>

class CLogger : public QObject
{
    Q_OBJECT
public:
    explicit CLogger(QString aFileName, QObject *parent = 0);
    virtual ~CLogger();

public:
    void setTitle(const QString & string);
    void addLog(const QString & string);
    CLogger &	operator<< ( const char * string );
    CLogger &	operator<< ( const QString & string );


//    CLogger &	operator<< ( QChar c );
//    CLogger &	operator<< ( signed short i );
//    CLogger &	operator<< ( float f );
//    CLogger &	operator<< ( char c );
//    CLogger &	operator<< ( unsigned short i );
//    CLogger &	operator<< ( signed int i );
//    CLogger &	operator<< ( unsigned int i );
//    CLogger &	operator<< ( signed long i );
//    CLogger &	operator<< ( unsigned long i );
//    CLogger &	operator<< ( qlonglong i );
//    CLogger &	operator<< ( qulonglong i );
//    CLogger &	operator<< ( double f );
//    CLogger &	operator<< ( const QByteArray & array );

private:
    QString time();
Q_SIGNALS:
    
public Q_SLOTS:

private:
    QTextStream         mTextStream;
    QFile               mFile;
    QString             mCR;        // Carriage Return
};

#endif // CLOGGER_H
