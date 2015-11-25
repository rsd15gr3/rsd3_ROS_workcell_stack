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

#include "ui_rsdPlugin.h"
#include "qtros.h"
#include "state_machine.h"

#include "kuka_ros/setConfiguration.h"
#include "Configurations.h"

using namespace Configurations;
using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;


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

public slots:
	void btnPressed();
	void stateChangedListener(const rw::kinematics::State& state);
    void newImage(cv::Mat);
    void updateConfiguration(kuka_ros::getConfiguration);
    void autoControlEnabled(bool);



signals:
	void quitNow();

private:
    void TestButtonsEnabled(bool _b);
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QtROS *_qtRos;
    state_machine *_state_machine;
	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	Device::Ptr _device;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
