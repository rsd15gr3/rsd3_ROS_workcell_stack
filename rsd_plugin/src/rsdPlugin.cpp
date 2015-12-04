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

//Q ConfigInit = Q(6,0.0,0.0,0.0,0.0,0.0,0.0);
//Q ConfigImgCapture = Q(6,1.420,0.679,0.360,0.699,-1.070,0.890);
//Q ConfigImgCapture = Q(6,1.424,0.681,0.363,0.731,-1.089,0.865);
//Q ConfigImgCapture = Q(6,1.453,0.818,0.370,1.154,-1.224,0.458);
//Q ConfigDropoff = Q(6,0.207,1.584,-0.738,0.000,-0.847,0.207);

bool init = false;

rsdPluginPlugin::rsdPluginPlugin():
    RobWorkStudioPlugin("rsdPlugin plugin", QIcon(":/pa_icon.png"))
{
	setupUi(this);
	char** argv = NULL;
    int argc = 0;
    ros::init(argc, argv,"rsdPlugin_plugin");


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




    connect(this, SIGNAL(quitNow()), _qtRos, SLOT(quitNow()));

    connect(_qtRos,SIGNAL(autoControlEnabledSignal(bool)),this,SLOT(autoControlEnabled(bool)));

    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(_qtRos, SIGNAL(newImage(cv::Mat)), this, SLOT(newImage(cv::Mat)));

    qRegisterMetaType<kuka_ros::getConfiguration>("kuka_ros::getConfiguration");
    connect(_qtRos,SIGNAL(updateConfiguration(kuka_ros::getConfiguration)), this, SLOT(updateConfiguration(kuka_ros::getConfiguration)));


    //connect(this, SIGNAL(quitNow()), _state_machine, SLOT(quitNow()));


    this->TestButtonsEnabled(false);
     //

}

rsdPluginPlugin::~rsdPluginPlugin()
{
}

void rsdPluginPlugin::autoControlEnabled(bool _autoenabled){
    if(!_btn0->isEnabled()){
    log().info() << "automode enabled: " << _autoenabled << "\n";
    _state_machine->SetIdle(_autoenabled);
    }
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
    _btn9->setEnabled(_b);
    _btn10->setEnabled(_b);
    _btn11->setEnabled(_b);
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

void rsdPluginPlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&rsdPluginPlugin::stateChangedListener, this, _1), this);
}

void rsdPluginPlugin::open(WorkCell* workcell)
{
    _wc = workcell;
    _state = _wc->getDefaultState();
    _device = _wc->findDevice("KR6");
    _state_machine = new state_machine(_wc,_state,_device);
    _qtRos->start();

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
         //disable test buttons and start statemachine
        connect(_qtRos,SIGNAL(newOrder(int)),_state_machine,SLOT(newOrder(int)));
        _state_machine->start();
        _btn0->setEnabled(false);
        //_state_machine->SetIdle(false); //start the statemachine
    } /*else if(obj==_btn1){
        log().info() << "Button 1 pressed (go to ImgCapture pos)\n";
        this->moveToImgCapture();
    } else if(obj==_btn2){
        log().info() << "Button 2 pressed (go to Dropoff pos)\n";
        this->SetConfiguration(ConfigDropoff);
    } else if(obj==_btn3){
        log().info() << "Button 3 pressed(go to zero pos)\n";
        this->moveToInit();
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
    }*/
}

void rsdPluginPlugin::stateChangedListener(const State& state) {
    _state = state;
}

Q_EXPORT_PLUGIN(rsdPluginPlugin);
