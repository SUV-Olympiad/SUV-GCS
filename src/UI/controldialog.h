#ifndef CONTROLDIALOG_H
#define CONTROLDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QSignalMapper>
#include <QMap>
#include <QQueue>
#include <QComboBox>

class CManager;

namespace Ui {
class CControlDialog;
}

class CControlDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CControlDialog(CManager* aManager, QWidget *parent = 0);
    ~CControlDialog();

public:

private:
    void initNodeLayout();
    void runDeploy(int aNodeID);

    bool shouldRetry(int agentId);
    void changeButtonColor(int agentId, QColor color);
    void addNextAgentToQueue(int agentId);
    float targetXYDistance(int mAgentId);
    void addHistory(QString text, int aId);

private Q_SLOTS:
    void onNodeClicked(int aId);
    void onAllLand();
    void updateStatus();
    void updateTypeCombobox(QString aType);

    void on_armButton_clicked();

    void on_takeoffButton_clicked();

    void on_landButton_clicked();

    void on_disarmButton_clicked();

    void on_upButton_clicked();

    void on_downButton_clicked();

    void on_minusYButton_clicked();

    void on_plusYButton_clicked();

    void on_minusXButton_clicked();

    void on_plusXButton_clicked();

    void on_pushButton_clicked();

private:
    void showEvent(QShowEvent *event);
    void closeEvent(QCloseEvent *event);
    void hideEvent(QHideEvent *event);

private:
    Ui::CControlDialog *ui;

    const float DEPLOY_ALTITUDE = 1.0;
    const float INTERVAL = 2.0;
    const int COLUMN_NODE = 10;
    const int STATUS_INITIAL = 0;
    const int STATUS_ARM = 1;
    const int STATUS_TAKEOFF = 2;
    const int STATUS_MOVE = 3;
    const int STATUS_LAND = 4;
    const int STATUS_DISARM = 5;
    const int STATUS_ARRIVED = 6;
    const int STATUS_FAIL = 7;
    const int TIME_TO_STABLIZE = 5; // how many seconds to determine the agent is stabilized or not
    const int MAX_RETRY_COUNT = 5;

    CManager*           mManager;
    int                 maxId = 0;
    QPushButton**       mNodeButton;
    QComboBox**         mNodeCombobox;
    QLabel**            mNodeLabel;
    QLabel*             mSelectedNodeLabel;
    QSignalMapper*      mSignalMapper;
    QTimer				mTimer;
    QMap<int, int>      mButtonIndexMap;       // <node id, button index>
    int                 currentSelectedNode = 0;
    int                 mNumAgent;
    QQueue<int>         queueToFlight;
    int**               currentStatus;
    int**               retryCount;
    float**             currentX;
    float**             currentY;
    float**             currentZ;
    float**             targetX;
    float**             targetY;
    float***            prevX;
    float***            prevY;
    float***            prevZ;
    int**               prevIndex;
};

#endif // CONTROLDIALOG_H
