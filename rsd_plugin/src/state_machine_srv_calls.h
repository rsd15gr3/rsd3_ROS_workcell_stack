#ifndef QT_state_machine_srv_calls_H
#define QT_sstate_machine_srv_calls_H
#include <ros/ros.h>
#include "../../WSG50/wsg_50_common/srv_gen/cpp/include/wsg_50_common/Move.h"
#include "kuka_ros/getConfiguration.h"
#include "kuka_ros/setConfiguration.h"
#include "kuka_ros/getIsMoving.h"
#include "brick_check/check_brick.h"
#include "plc_comm/plc_service.h"
#include <std_msgs/Bool.h>
#include <QObject>
#include <rw/rw.hpp>
#include "std_srvs/Empty.h"
#include "brick_detection/bricks.h"
#include "Configurations.h"
#include <rw/models/Device.hpp>
#include <rw/rw.hpp>
#include <rw/models/Device.hpp>
#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

using namespace std;
using namespace rw;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::invkin;

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::sensor;

using namespace rw::trajectory;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::math;
using namespace rw::invkin;

struct brick {
    double x;
    double y;
    double angle;
    int color;
};

class state_machine_srv_calls {
public:
    state_machine_srv_calls();
    state_machine_srv_calls(rw::models::WorkCell::Ptr _wc, rw::kinematics::State _state, Device::Ptr _device);
    bool openGripper();
    bool closeGripper(bool BrickOnSide = false, double speedPct = 100);
    bool brickPresent(int color);
    bool robotMoving();
    bool closeToConfig();
    bool moveTo(Q _q);
    bool backOffBrick();
    bool moveToBrickColor(int color);
    std::vector<brick> getBricks();
    bool moveToBrick(double xPos, double yPos,double yRot);
    bool checkPick();
    bool conveyorBelt(int speed = 2, bool activate = true, bool forward = true);
    bool conveyorBeltStop();
    ros::NodeHandle n;
    ros::ServiceClient clientGetBricks;
    ros::ServiceClient clientConveyorBelt;

private:
    rw::models::WorkCell::Ptr wc;
    rw::kinematics::State state;
    Device::Ptr device;
    std::vector<Q>  pickupPath;
    bool checkQcontraints(Q q);
    std::vector<Q> JacobianIKSover_50trys_Contrainted(Transform3D<> _target);
    std::vector<Q> calulatePickupPath(Transform3D<> start, double m_length = 0.05);
    Q lastSetQ;
    int lastPickBrickColor;





};
#endif

