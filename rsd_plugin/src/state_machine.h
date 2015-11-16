/** 
 * QtThread based class encapsulating the ros basics,
 * i.e., init, node handle creation, spining and quitting.
 * To quit via qt, connect the quitNow slot to, e.g., 
 * the aboutToQuit-Signal of qapplication.
 */
#ifndef QT_state_machine_H
#define QT_state_machine_H
#include <ros/ros.h>
#include "kuka_ros/getConfiguration.h"
#include "kuka_ros/setConfiguration.h"
#include <std_msgs/Bool.h>
#include <QThread>
#include <QObject>
#include <mutex>

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
        bool backOffBrick();
        bool openGripper();
        bool closeGripper(bool BrickOnSide = false, double speedPct = 100);

    public slots:
        void autoControlEnabled(bool);
        void quitNow();

    private:
        bool timeout;
        bool idle;
        mutex idleMutex;
        bool quitfromgui;

};
#endif

