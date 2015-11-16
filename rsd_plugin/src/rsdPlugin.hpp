#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <rw/rw.hpp>
#include <rw/models/Device.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include "../../rsd_plugin-build/ui_rsdPlugin.h"
#include "qtros.h"
#include "state_machine.h"

#include "kuka_ros/setConfiguration.h"

using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;

struct brick {
    double x;
    double y;
    double angle;
    int color;
};


class rsdPluginPlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
	rsdPluginPlugin();
	virtual ~rsdPluginPlugin();
	virtual void open(rw::models::WorkCell* workcell);
	virtual void close();
	virtual void initialize();

private slots:
    //UI stuff
	void btnPressed();
    void timer(); //TODO: Remove the timer
	void stateChangedListener(const rw::kinematics::State& state);
    void newImage(cv::Mat);
    void updateConfiguration(kuka_ros::getConfiguration);


    void autoControlEnabled(bool);
    void TestButtonsEnabled(bool _b);
    //statemachine signals
    void moveToImgCapture();
    void moveToDropoff();
    bool backOffBrick();
    bool openGripper();
    bool closeGripper(bool BrickOnSide = false, double speedPct = 100);



signals:
    void setConfigurationAuto(kuka_ros::setConfiguration _q_srv);
	void quitNow();

private:
    //void TestButtonsEnabled(bool _b);
    //bool closeGripper(bool BrickOnSide = false, double speedPct = 100);
    //void moveToImgCapture();
    //bool openGripper();
    //bool backOffBrick();
    //void moveToDropoff();
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    bool SetConfiguration(Q _q);

    bool checkQcontraints(Q _q);
    bool moveToBrickColor(int color);
    bool moveToBrick(double xPos = 0.0, double yPos = 0.0, double yRot = 0.0);    

    std::vector<brick> getBricks();
    std::vector<Q> JacobianIKSover_50trys_Contrainted(Transform3D<> target);
    std::vector<Q> calulatePickupPath(Transform3D<> start, double m_length = 0.05);
	QTimer* _timer;
    QtROS *_qtRos;
    state_machine _state_machine;
	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	Device::Ptr _device;
    std::vector<Q>  pickupPath;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
