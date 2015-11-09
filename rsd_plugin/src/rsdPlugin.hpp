#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP
#include "../../rsd_plugin-build/ui_rsdPlugin.h"
#include "qtros.h"
#include <ros/ros.h>
#include <opencv/cv.h>
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
	void btnPressed();
	void timer();
	void stateChangedListener(const rw::kinematics::State& state);
    void newImage(cv::Mat);
    void updateConfiguration(kuka_rsi::getConfiguration);

signals:
	void quitNow();

private:
    void TestButtonsEnabled(bool _b);
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    bool SetConfiguration(Q _q);
    bool checkQcontraints(Q _q);
    void moveToImgCapture();
    bool moveToBrickColor(int color);
    bool moveToBrick(double xPos = 0.0, double yPos = 0.0, double yRot = 0.0);
    bool closeGripper(bool BrickOnSide = false, double speedPct = 100);
    bool openGripper();
    std::vector<brick> getBricks();
    bool backOffBrick();
    void moveToDropoff();
    std::vector<Q> JacobianIKSover_50trys_Contrainted(Transform3D<> target);
    std::vector<Q> calulatePickupPath(Transform3D<> start, double m_length = 0.05);
	QTimer* _timer;
    QtROS *_qtRos;
	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	Device::Ptr _device;
    std::vector<Q>  pickupPath;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
