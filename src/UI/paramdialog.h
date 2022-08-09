#ifndef CPARAMDIALOG_H
#define CPARAMDIALOG_H

#include <QTimer>
#include "ui_paramdialog.h"
#include "vehicle.h"

class CManager;

class CParamDialog : public QDialog, public Ui::ParamDialog
{
	Q_OBJECT

public:
	CParamDialog(CManager* aManager, QWidget * parent = 0);

public:
    void updateNode();

private Q_SLOTS:
    void tableChanged(int aRow, int aCol);
    void tableClicked(int aRow, int aCol);
    void updateParam();
    void updateUI();

    void on_initializeButton_clicked();

private:
	void initDialog();
    void showEvent(QShowEvent *event);
    void closeEvent(QCloseEvent *event);
    void hideEvent(QHideEvent *event);
    void setParam(IVehicle *agent, QString key, QVariant::Type type, QVariant value);

private:
	CManager*                           mManager;    
    QTimer                              mTimer;

    bool                                mClicked;
    int                                 mClickedRow;
    int                                 mClickedCol;

    bool                                mCheckProgress;
    int                                 mParamChangedRetryCount;
    QString                             mChangedParam;
    QString                             mChangedParamValue;

};

#endif // CPARAMDIALOG_H
