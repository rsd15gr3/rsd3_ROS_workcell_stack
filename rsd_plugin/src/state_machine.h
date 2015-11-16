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

class state_machine : public QThread {
  Q_OBJECT

  public:
    state_machine();
    void run();
  public slots:
    void quitNow();


  private:
	bool quitfromgui;

};
#endif

