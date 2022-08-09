#ifndef EMSCENARIODIALOG_H
#define EMSCENARIODIALOG_H

#include <QDialog>
#include <QTimer>
#include <QDateTime>
// #include <QtMultimedia/QMediaPlayer>

class CManager;

namespace Ui {
class CEmScenarioDialog;
}

class CEmScenarioDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CEmScenarioDialog(CManager* aManager, QWidget *parent = 0);
    ~CEmScenarioDialog();

public:
    void startTimer();
	void stopMusic();

private Q_SLOTS:
    void updateUI();
    void onDone(int result);
    void onReserve();
    void onSetConfig();
    void onReset();
	void playMusic();

private:
    Ui::CEmScenarioDialog *ui;

    CManager*               mManager;
    QTimer					mTimer;
	QTimer					mMusicTimer;

    float                   mStartTime;         // reserved scenario start time
    bool                    mOnReserve;
    bool                    mOnSetConfig;
    QDateTime               mReservedClickTime;      // the time when the reserved push button is pressed (second)

	// QMediaPlayer            mScenarioMusic;

};

#endif // EMSCENARIODIALOG_H
