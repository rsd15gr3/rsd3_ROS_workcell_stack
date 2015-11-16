#include "rsdPlugin.hpp"
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <rw/models/Device.hpp>
#include <rws/RobWorkStudio.hpp>
#include <QPushButton>
#include <rw/kinematics.hpp>

#include "kuka_ros/setConfiguration.h" //set config
#include "../../WSG50/wsg_50_common/srv_gen/cpp/include/wsg_50_common/Move.h"
#include "brick_detection/bricks.h"
#include "std_srvs/Empty.h"

#include <rw/common.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <unistd.h> //test


using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;

using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::math;
using namespace rw::invkin;



using namespace rws;

Q ConfigInit = Q(6,0.0,0.0,0.0,0.0,0.0,0.0);
Q ConfigImgCapture = Q(6,1.420,0.679,0.360,0.699,-1.070,0.890);
//Q ConfigImgCapture = Q(6,1.424,0.681,0.363,0.731,-1.089,0.865);
//Q ConfigImgCapture = Q(6,1.453,0.818,0.370,1.154,-1.224,0.458);
Q ConfigDropoff = Q(6,0.207,1.584,-0.738,0.000,-0.847,0.207);


rsdPluginPlugin::rsdPluginPlugin():
    RobWorkStudioPlugin("rsdPlugin plugin", QIcon(":/pa_icon.png"))
{
	setupUi(this);
	char** argv = NULL;
    int argc = 0;
    ros::init(argc, argv,"rsdPlugin_plugin");

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn2    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn3    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn4    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn5    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn6    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn7    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn8    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn9    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn10    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn11    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    _qtRos = new QtROS();
    _state_machine = new state_machine();

    connect(this, SIGNAL(quitNow()), _qtRos, SLOT(quitNow()));

    connect(_qtRos,SIGNAL(autoControlEnabledSignal(bool)),this,SLOT(autoControlEnabled(bool)));

    qRegisterMetaType<kuka_ros::setConfiguration>("kuka_ros::setConfiguration");
    connect(this, SIGNAL(setConfigurationAuto(kuka_ros::setConfiguration)),_qtRos, SLOT(setConfigurationAuto(kuka_ros::setConfiguration)));

    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(_qtRos, SIGNAL(newImage(cv::Mat)), this, SLOT(newImage(cv::Mat)));

    qRegisterMetaType<kuka_ros::getConfiguration>("kuka_ros::getConfiguration");
    connect(_qtRos,SIGNAL(updateConfiguration(kuka_ros::getConfiguration)), this, SLOT(updateConfiguration(kuka_ros::getConfiguration)));

    connect(this, SIGNAL(quitNow()), _state_machine, SLOT(quitNow()));

     _qtRos->start();
}

rsdPluginPlugin::~rsdPluginPlugin()
{
}

void rsdPluginPlugin::autoControlEnabled(bool _autoenabled){
    log().info() << "automode enabled: " << _autoenabled << "\n";
}

std::vector<brick> rsdPluginPlugin::getBricks()
{
    std::vector<brick> bricks;
    brick_detection::bricks _brick_srv;
    brick_detection::bricks::Request req;
    ros::service::call("/brick_detector/getBricks",_brick_srv);
    log().info() << _brick_srv.response << "/n";
    if(!(_brick_srv.response.x.size() > 0))
    {
        log().info() << "Error: Failed to call brick service!\n";
        return bricks;
    }

    if(_brick_srv.response.x.size() > 0)
    {
        for(unsigned int i = 0; i !=_brick_srv.response.x.size(); i++)
        {
            brick tmpBrick;
            tmpBrick.color = _brick_srv.response.type[i];
            tmpBrick.x = _brick_srv.response.x[i];
            tmpBrick.y = _brick_srv.response.y[i];
            tmpBrick.angle = _brick_srv.response.angle[i];
            bricks.push_back(tmpBrick);
        }
    }
    log().info() << "Found: " << _brick_srv.response.x.size() << " bricks\n";
    return bricks;
}

void rsdPluginPlugin::TestButtonsEnabled(bool _b)
{
    _btn1->setEnabled(_b);
    _btn2->setEnabled(_b);
    _btn3->setEnabled(_b);
    _btn4->setEnabled(_b);
    _btn5->setEnabled(_b);
    _btn6->setEnabled(_b);
    _btn7->setEnabled(_b);
    _btn8->setEnabled(_b);
    _xPosdoubleSpinBox->setEnabled(_b);
    _yPosdoubleSpinBox->setEnabled(_b);
    _yRotdoubleSpinBox->setEnabled(_b);

}

void rsdPluginPlugin::newImage(cv::Mat image){

        if(image.rows > 0 and image.cols > 0)
	{
        cv::Mat temp; // make the same cv::Mat
        cv::cvtColor(image, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
        QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
        QImage dest2(dest);
        dest2.detach(); // enforce deep copy

        QPixmap p = QPixmap::fromImage(dest2);
        unsigned int maxW = 400;
        unsigned int maxH = 300;
        _label_RobotCamImage->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
	}
}

void rsdPluginPlugin::updateConfiguration(kuka_ros::getConfiguration config){

   Q qq = Q(6,config.response.q[0]*0.017453,config.response.q[1]*0.017453,config.response.q[2]*0.017453,
           config.response.q[3]*0.017453,config.response.q[4]*0.017453,config.response.q[5]*0.017453);
   _device->setQ(qq,_state);
   getRobWorkStudio()->setState(_state);
}

bool rsdPluginPlugin::SetConfiguration(Q _q) {
    if(_q.size() == 6)
    {
        log().info() << "Setting configuration: " << _q << "\n";
        kuka_ros::setConfiguration _q_srv;
        _q_srv.request.q[0] = _q[0];
        _q_srv.request.q[1] = _q[1];
        _q_srv.request.q[2] = _q[2];
        _q_srv.request.q[3] = _q[3];
        _q_srv.request.q[4] = _q[4];
        _q_srv.request.q[5] = _q[5];
        emit setConfigurationAuto(_q_srv);
        //ros::service::call("/KukaNode/SetConfiguration",_q_srv);
        return true;
    }
    else
        log().info() << "Error: Q.size != 6 \n";
        return false;
}


void rsdPluginPlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&rsdPluginPlugin::stateChangedListener, this, _1), this);
}


void rsdPluginPlugin::open(WorkCell* workcell)
{
    _wc = workcell;
	_state = _wc->getDefaultState();
    _device = _wc->findDevice("KR6");
    //log().info() << "Found device KR6";
}


void rsdPluginPlugin::close() {
    _wc = NULL;
}

cv::Mat rsdPluginPlugin::toOpenCVImage(const Image& img) {
	cv::Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void rsdPluginPlugin::btnPressed() {


	QObject *obj = sender();
	if(obj==_btn0){
        log().info() << "Button 0 pressed ()\n";
        this->TestButtonsEnabled(false); //disable test buttons and start statemachine
        _state_machine->start(); //start the statemachine
    } else if(obj==_btn1){
        log().info() << "Button 1 pressed (go to ImgCapture pos)\n";
        this->SetConfiguration(ConfigImgCapture);
    } else if(obj==_btn2){
        log().info() << "Button 2 pressed (go to Dropoff pos)\n";
        this->SetConfiguration(ConfigDropoff);
    } else if(obj==_btn3){
        log().info() << "Button 3 pressed(go to zero pos)\n";
        this->SetConfiguration(ConfigInit);
    } else if(obj==_btn4){
        log().info() << "Button 4 pressed (move to brick)\n";
        this->moveToBrick(_xPosdoubleSpinBox->value(),_yPosdoubleSpinBox->value(),_yRotdoubleSpinBox->value());
    } else if(obj==_btn5){
        log().info() << "Button 5 pressed (open gripper)\n";
        this->openGripper();
    } else if(obj==_btn6){
        log().info() << "Button 6 pressed (close (gripper)\n";
        this->closeGripper();
    } else if(obj==_btn7){
        log().info() << "Button 7 pressed (move back from grasp)\n";
        this->backOffBrick();
    } else if(obj==_btn8){
        log().info() << "Button 8 pressed (find bricks)\n";
        this->getBricks();
    } else if(obj==_btn9){
        log().info() << "Button 9 pressed (move to blue brick)\n";
        this->moveToBrickColor(2);
    } else if(obj==_btn10){
        log().info() << "Button 8 pressed (move to red bricks)\n";
        this->moveToBrickColor(0);
    } else if(obj==_btn11){
        log().info() << "Button 8 pressed (move to yellow bricks)\n";
        this->moveToBrickColor(1);
    }
}

void rsdPluginPlugin::moveToImgCapture(){
    this->SetConfiguration(ConfigImgCapture);
}

bool rsdPluginPlugin::moveToBrickColor(int color) {
    std::vector<brick> _bricks = this->getBricks();

    for(brick _brick : _bricks)
    {
        if(_brick.color == color)
        {
            log().info() << "Moving to brick: \n"
                         << "Color: " << color << "\n"
                         << "X: " << _brick.x << "\n"
                         << "Y: " << _brick.y << "\n"
                         << "Angle: " << _brick.angle << "\n";
            this->moveToBrick(_brick.x,_brick.y,_brick.angle);
            return true;
        }
    }
    return false;

}



                                 //xpos +-0.10 //ypos +-0.075(checked) // //yRos +- pi/2
bool rsdPluginPlugin::moveToBrick(double xPos, double yPos,double yRot) {
    if(xPos > 0.101 || xPos < -0.101 || yPos > 0.0751 || yPos < -0.0751 || yRot > 1.581 || yRot < -1.581)
    {
        log().info() << "pickupBrick: xPos, yPos or yRot is breaking contraints!\n";
        return false;
    }

    //get frames and calculate Robot->conveyotbelt trasformation
    Frame* conveyorbeltPickupCenter = _wc->findFrame("conveyorBeltPickupCenter");
    Transform3D<> _conveyorbeltTopTransformation;
    _conveyorbeltTopTransformation = conveyorbeltPickupCenter->wTf(_state);

    Frame* Robot = _wc->findFrame("Robot");
    Transform3D<> _RobotTransformation;
    _RobotTransformation = Robot->wTf(_state);

    Transform3D<> _invRobotTransformation = rw::math::inverse(_RobotTransformation);
    Transform3D<> _RobotConveyorTransformation;
    _invRobotTransformation.multiply(_invRobotTransformation,_conveyorbeltTopTransformation,_RobotConveyorTransformation);

    Vector3D<> PosOverBrick(-xPos, -yPos, 0.221); //22.1 cm over frame
    RPY<> RotBrickRPY(-yRot,0.0,0.0);
    Rotation3D<> RotBrick = RotBrickRPY.toRotation3D();
    Transform3D<> OverBrickTransformation(PosOverBrick,RotBrick);
    OverBrickTransformation.multiply(_RobotConveyorTransformation,OverBrickTransformation,OverBrickTransformation);

    pickupPath = this->calulatePickupPath(OverBrickTransformation);
    if(!pickupPath.size() > 0)
    {
        log().info() << "pickupBrick: Failed to pickup brick!\n";
        return false;
    }
    //Check contraints
    for (Q q : pickupPath )
    {
        if(!checkQcontraints(q))
        {
            log().info() << "pickupBrick: Failed! Found path breaks contraints!\n";
            return false;
        }
    }
    //move robot
    for(Q q : pickupPath )
    {
        this->SetConfiguration(q);
    }

    ///TODO: close gripper

    return true;

}

bool rsdPluginPlugin::backOffBrick(){
    if (!this->pickupPath.size() > 0)
    {
        log().info() << "backOffBrick: Failed! pickupPath vector is empty!!\n";
        return false;
    }
    else
    {
        /*for (unsigned i = this->pickupPath.size(); i-- > 0; )
        {
            this->SetConfiguration(pickupPath.at(i));
        }*/
        this->SetConfiguration(pickupPath.front());
        this->SetConfiguration(ConfigImgCapture);
        return true;
    }
}

void rsdPluginPlugin::moveToDropoff(){
    this->SetConfiguration(ConfigDropoff);
}

bool rsdPluginPlugin::checkQcontraints(Q q){
    if(q[0] < 1.300 || q[0] > 1.85 || q[3] > 1.2 || q[3] < -1.2 || q[1] > 1.88)
        return false;
    else
        return true;
}

//pcikup planning
std::vector<Q> rsdPluginPlugin::calulatePickupPath(Transform3D<> start, double m_length){

    std::vector<Q> path;
    std::vector<Q> solutions = this->JacobianIKSover_50trys_Contrainted(start);
    if(solutions.size() > 0)
    {
        path.push_back(solutions.at(0));//add step
    }
    else
    {
        log().info() << "calulatePickupPath: JacobianIKSover_50trys failed to find configuration for passed transforamation!\n"
                     << "Is the transformation passed to calulatePickupPath valid??\n";
        return solutions; //return same error as JacobianIKSolver if it fails
    }

    //calulate transform for moving -0.005m along the Z axis
    /*Vector3D<> PosStepMultipli(0.0, 0.0, -0.005);
    Rotation3D<> RotStepMultipli( 1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0);
    Transform3D<> StepMultipli(PosStepMultipli,RotStepMultipli);

    double DistMoved = 0.000;
    Transform3D<> StepTransform = start;
    Transform3D<> nextStepTransform = start;

    while(DistMoved < m_length-0.006) //keep calulating steps down, until the goal is under 0.011m away
    {
        StepTransform.multiply(StepTransform,StepMultipli,nextStepTransform);
        std::vector<Q> solutions = this->JacobianIKSover_50trys(nextStepTransform);
        StepTransform = nextStepTransform;
        if(solutions.size() > 0)
            path.push_back(solutions.at(0));//add step
        else
        {
            log().info() << "calulatePickupPath: JacobianIKSover_50trys failed!\n"
                         << "Is the disired movement along the z axis out of the robot reach??\n";
            return solutions; //return same error as JacobianIKSolver if it fails
        }
        DistMoved += 0.005;
    }*/



    //calulate final pos configuration
    Vector3D<> PosFinalMultipli(0.0, 0.0, -m_length);
    Rotation3D<> RotFinalMultipli( 1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0);
    Transform3D<> StepFinalMultipli(PosFinalMultipli,RotFinalMultipli);
    Transform3D<> FinalTransform;
    StepFinalMultipli.multiply(start,StepFinalMultipli,FinalTransform);
    std::vector<Q> solutions2 = this->JacobianIKSover_50trys_Contrainted(FinalTransform);
    if(solutions2.size() > 0)
        path.push_back(solutions2.at(0));//add step
    else
    {
        log().info() << "calulatePickupPath: JacobianIKSover_50trys failed to find the final configuration!\n"
                     << "Is the disired movement along the z axis out of the robot reach??\n";
        return solutions2; //return same error as JacobianIKSolver if it fails
    }

    return path;
}

std::vector<Q> rsdPluginPlugin::JacobianIKSover_50trys_Contrainted(Transform3D<> _target){
    std::vector<Q> solutions;
    for(int i = 0; i<50; i++)
    {
        JacobianIKSolver solver(_device, _state);
        solver.setCheckJointLimits(true);
        solutions = solver.solve(_target, _state);
        if(solutions.size() > 0)
        {
            //remove solutions that break the contraints
            for(std::vector<Q>::iterator it = solutions.begin(); it !=solutions.end(); ++it)
            {
                if(!this->checkQcontraints(*it)){
                    solutions.erase(it);
                }
            }
            //if there is any solutions left, return them
            if(solutions.size() > 0)
                return solutions;
        }
    }
    log().info() << "JacobianIKSover_50trys: Failed to find configuration!\n";
    return solutions;
}

bool rsdPluginPlugin::closeGripper(bool BrickOnSide, double speedPct)
{
    bool res = true;
    wsg_50_common::Move srv;
    srv.request.speed = speedPct;
    if(BrickOnSide)
        srv.request.width = 30.0;//not calibrated
    else
        srv.request.width = 22.0;
    //ROS_INFO("Gripper trying to move to width =%d at speed=%d", (int)srv.request.width,(int)srv.request.speed);
    if (ros::service::call("/wsg_50/move",srv))
    {
      if(srv.response.error > 0) {
          //error from gripper... try ack and move again
          log().info() << "Gripper error: " << (int)srv.response.error << "\n" <<
                          "Acknowledgeing and trying to close the gripper again..." << "\n";
          std_srvs::Empty AckSrv;
          ros::service::call("/wsg_50/ack",AckSrv);
          if (ros::service::call("/wsg_50/move",srv))
          {
              if(srv.response.error > 0) {
                  log().info() << "Gripper error: " << (int)srv.response.error << "\n";
                  return false;
              }
              else {
                   log().info() << "Gripper moved to width = " << (int)srv.request.width << " at speed=" << (int)srv.request.speed << "\n";
                   return true;
              }
          }
          return false;
      } else {
           log().info() << "Gripper moved to width = " << (int)srv.request.width << " at speed=" << (int)srv.request.speed << "\n";
           return true;
      }
   }
    else
    {
        return false;
    }
    return res;
}

bool rsdPluginPlugin::openGripper()
{
    bool res = true;
    wsg_50_common::Move srv;
    srv.request.speed = 100;
    srv.request.width = 40;
    //ROS_INFO("Gripper trying to move to width =%d at speed=%d", (int)srv.request.width,(int)srv.request.speed);
    if (ros::service::call("/wsg_50/move",srv))
    {
      if(srv.response.error > 0) {
          //error from gripper... try ack and move again
          log().info() << "Gripper error: " << (int)srv.response.error << "\n" <<
                          "Acknowledgeing and trying to open the gripper again..." << "\n";
          std_srvs::Empty AckSrv;
          ros::service::call("/wsg_50/ack",AckSrv);
          if (ros::service::call("/wsg_50/move",srv))
          {
              if(srv.response.error > 0) {
                  log().info() << "Gripper error: " << (int)srv.response.error << "\n";
                  return false;
              }
              else {
                   log().info() << "Gripper moved to width = " << (int)srv.request.width << " at speed=" << (int)srv.request.speed << "\n";
                   return true;
              }
          }
          return false;
      } else {
         log().info() << "Gripper open" << "\n";
      }
   }
    else
    {
        res = false;
    }
    return res;
}

void rsdPluginPlugin::timer() {
        _timer->stop();
}

void rsdPluginPlugin::stateChangedListener(const State& state) {
    _state = state;
}

Q_EXPORT_PLUGIN(rsdPluginPlugin);
