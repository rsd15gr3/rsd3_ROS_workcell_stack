/** 
 * QtThread based class encapsulating the ros basics,
 * i.e., init, node handle creation, spining and quitting.
 * To quit via qt, connect the quitNow slot to, e.g., 
 * the aboutToQuit-Signal of qapplication.
 */
#ifndef QT_state_machine_H
#define QT_state_machine_H
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <QThread>
#include <QObject>
#include <QMessageBox>
#include <QTimer>
#include <mutex>
#include "state_machine_srv_calls.h"


using namespace std;

class state_machine : public QThread {
  Q_OBJECT

    public:
        state_machine();
        void run();
        void SetIdle(bool idle);
        bool GetIdle();

    signals:
        void moveToImgCapture();
        void moveToDropoff();
        void moveToInit();
        void moveToBrick(int color);
        bool backOffBrick();

    public slots:
        void autoControlEnabled(bool);
        void quitNow();
        void gripperTimeslot();

    private:
        bool timeout;
        bool idle;
        mutex idleMutex;
        bool quitfromgui;
        int state;
        int old_state;
        int position;
        state_machine_srv_calls srv_call;
        QTimer *timer;
        QMessageBox msgBox;

};
#endif

