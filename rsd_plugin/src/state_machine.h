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
#include "Configurations.h"

using namespace std;

class state_machine : public QThread {
  Q_OBJECT

    public:
        state_machine(rw::models::WorkCell::Ptr _wc, rw::kinematics::State _state, Device::Ptr _device);
        void run();
        void SetIdle(bool idle);
        bool GetIdle();

    signals:
        void Error();
    public slots:
        void autoControlEnabled(bool);
        void newOrder(int color);
        void quitNow();
        void gripperTimeslot();
        void ErrorAck();

    private:
        bool timeout;
        bool idle;
        bool ErrorFlag;
        bool quitfromgui;
        bool moving;
        bool beltRunning;
        int state;
        int old_state;
        int position;
        int failCounter;

        bool stateprinted;
        mutex idleMutex;
        state_machine_srv_calls srv_call;
        QTimer *timer;
        QMessageBox msgBox;

};
#endif

