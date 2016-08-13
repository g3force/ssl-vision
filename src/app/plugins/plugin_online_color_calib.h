/*
 * plugin_online_color_calib.h
 *
 *  Created on: Jul 19, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_
#define SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_

#include <visionplugin.h>
#include "cmvision_region.h"
#include "lut3d.h"
#include "field.h"
#include "camera_calibration.h"
#include "plugin_visualize.h"
#include "VarTypes.h"
#include "messages_robocup_ssl_detection.pb.h"

#include <chrono>
#include <lwpr.hh>

#include <QThread>
#include <QObject>
#include <QString>

#include <mutex>
#include <condition_variable>

class RobotRegions {
public:
	double time;
	vector3d pos;
	double orientation;
	std::vector<CMVision::Region> regions;
};

class LocStamped {
public:
	double time;
	pixelloc loc;
	int clazz;
	double prop;
};

class BotPosStamped {
public:
	double time;
	vector3d pos;
	double orientation;
};

class AngleRange {
public:
	double min;
	double max;
};

class ClazzProperties {
public:
	double minDist;
	double maxDist;
	double height;
	double radius;
	int color;
	int nAngleRanges;
	AngleRange angleRanges[2];
};

class WorkerInput {
public:
	long long int number;
	RawImage image;
	SSL_DetectionFrame detection_frame;
	std::vector<CMVision::Region> regions;
};

class Worker: public QObject {
	Q_OBJECT
public:
	Worker(LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field);
	virtual ~Worker();
	virtual void update(FrameData * frame);
	virtual void CopytoLUT(LUT3D *lut);
	virtual void GetLocs(std::vector<LocStamped>& locs);
	virtual void resetModel();

	YUVLUT local_lut;
	LUT3D * global_lut;

	VarBool * _v_lifeUpdate;
	VarBool * _v_removeOutlierBlobs;

	std::vector<LocStamped*> locs;
	std::vector<ClazzProperties> cProp;
	std::vector<int> color2Clazz;

	bool globalLutUpdate;
public slots:
	void process();
signals:
	void finished();
	void error(QString err);

private:
	virtual void updateModel(RawImage& image, pixelloc& loc, int clazz);

	virtual void processRegions(const SSL_DetectionFrame * detection_frame, std::vector<CMVision::Region>& regions);
//	virtual void updateRobotRegions(const SSL_DetectionFrame * detection_frame,
//			std::vector<RobotRegions*>& robotRegionsList);
	virtual void updateBotPositions(const SSL_DetectionFrame * detection_frame,
			std::vector<BotPosStamped*>& botPoss);
	virtual void updateLocs(const SSL_DetectionFrame * detection_frame,
			std::vector<LocStamped*>& locs);

	virtual int getColorFromModelOutput(doubleVec& output);

	virtual void regionFieldDim(CMVision::Region* region, int clazz,
			double& width, double& height);
	virtual void regionDesiredPixelDim(CMVision::Region* region, int clazz,
			int& width, int& height);
	virtual bool isInAngleRange(vector3d& pField, int clazz,
			BotPosStamped* botPos);

	virtual void addRegion(const SSL_DetectionFrame * detection_frame,
			std::vector<LocStamped*>& locs, int clazz, float x, float y,
			double prop = 1);
	virtual void addRegionCross(const SSL_DetectionFrame * detection_frame,
			std::vector<LocStamped*>& locs, int clazz, int targetClazz,
			CMVision::Region* region, int width, int height, int exclWidth,
			int exclHeight, int offset);
	virtual void addRegionEllipse(const SSL_DetectionFrame * detection_frame,
			std::vector<LocStamped*>& locs, int clazz, int targetClazz,
			CMVision::Region* region, int width, int height, int exclWidth,
			int exclHeight, int offset);

	// synchronization
	std::mutex d_mutex;
	std::mutex mutex_input;
	std::mutex mutex_locs;
	std::mutex mutex_model;
	std::condition_variable d_condition;

	// input data
	WorkerInput input;

	// state
	const CameraParameters& camera_parameters;
	const RoboCupField& field;

	LWPR_Object* model;
	float robot_tracking_time;

	std::vector<BotPosStamped*> botPoss;
	std::vector<RobotRegions*> robotRegionsList;
};

class PluginOnlineColorCalib: public VisionPlugin {
	Q_OBJECT
public:
	PluginOnlineColorCalib(FrameBuffer * _buffer, LUT3D * lut,
			const CameraParameters& camera_params, const RoboCupField& field);
	virtual ~PluginOnlineColorCalib();

	virtual ProcessResult process(FrameData * data, RenderOptions * options);
	virtual VarList * getSettings();
	virtual string getName();

protected slots:
	void slotUpdateTriggered();
	void slotResetModelTriggered();
private:

	VarList * _settings;
	VarBool * _v_enable;
	VarBool * _v_debug;
	VarTrigger * _updGlob;
	VarTrigger * _resetModel;
	Worker* worker;

};

#endif /* SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_ */
