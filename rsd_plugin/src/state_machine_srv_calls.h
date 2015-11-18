#ifndef QT_state_machine_srv_calls_H
#define QT_sstate_machine_srv_calls_H
#include <ros/ros.h>
#include "../../WSG50/wsg_50_common/srv_gen/cpp/include/wsg_50_common/Move.h"
#include "kuka_ros/getConfiguration.h"
#include "kuka_ros/setConfiguration.h"
#include <std_msgs/Bool.h>
#include <QObject>

using namespace std;

class state_machine_srv_calls {
    public:
    bool openGripper();
    bool closeGripper(bool BrickOnSide = false, double speedPct = 100);
    bool brickPresent(int color);
    bool robotMoving();
};
#endif

