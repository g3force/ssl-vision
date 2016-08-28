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

#include "BlobDetector.h"

class LocStamped {
public:
	double time;
	pixelloc loc;
	int clazz;
};

class Loc {
public:
	uint16_t x,y;
	int8_t clazz;
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
	double time;
	RawImage image;
	std::vector<CMVision::Region> regions;
};

class Worker: public QObject {
	Q_OBJECT
public:
	Worker(LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field);
	virtual ~Worker();
	virtual void update(FrameData * frame);
	virtual void CopytoLUT(LUT3D *lut);
	virtual void ResetModel();
	virtual void updateBotPositions(const SSL_DetectionFrame * detection_frame);

	YUVLUT local_lut;
	LUT3D * global_lut;
	const CameraParameters& camera_parameters;

	VarBool * _v_lifeUpdate;
	VarBool * _v_removeOutlierBlobs;

	std::vector<ClazzProperties> cProp;
	std::vector<int> color2Clazz;
	std::mutex mutex_botPoss;
	std::mutex mutex_locs;
	std::vector<BotPosStamped*> botPoss;
	std::vector<Loc> locs_out;

	bool globalLutUpdate;
	bool running;
public slots:
	void process();
signals:
	void finished();
	void error(QString err);

private:
	virtual int getColorFromModelOutput(doubleVec& output);
	virtual void updateModel(RawImage& image, pixelloc& loc, int clazz);

	virtual void processRegions(RawImage& image, double time, std::vector<CMVision::Region>& regions);

	virtual BotPosStamped* findNearestBotPos(const vector3d& loc,	double* dist, BotPosStamped* exceptThisBot = 0);

	virtual void getRegionFieldDim(CMVision::Region* region, int clazz, double& width, double& height);
	virtual void getRegionDesiredPixelDim(CMVision::Region* region, int clazz,	int& width, int& height);
	virtual bool isInAngleRange(vector3d& pField, int clazz, BotPosStamped* botPos);

	virtual void updateLocs(double time);
	virtual LocStamped* findNearestLoc(const LocStamped& loc, double* dist, LocStamped* exceptThis = 0);
	virtual void addLoc(double time, int clazz, float x, float y);
	virtual void addRegionCross(double time, int clazz, int targetClazz,
			CMVision::Region* region, int width, int height, int exclWidth,
			int exclHeight, int offset);
	virtual void addRegionEllipse(double time, int clazz, int targetClazz,
			CMVision::Region* region, int width, int height, int exclWidth,
			int exclHeight, int offset);
	virtual void addRegionKMeans(RawImage * img, double time, int targetClazz,
			CMVision::Region* region, int width, int height, int offset);

	// synchronization
	std::mutex mutex_sync;
	std::mutex mutex_input;
	std::mutex mutex_model;
	std::condition_variable d_condition;

	// input data
	WorkerInput* input;
	WorkerInput inputData[2];
	int inputIdx;

	// state
	const RoboCupField& field;

	std::vector<LWPR_Object*> models;
	float robot_tracking_time;
	float loc_tracking_time;
	int max_regions;
	int max_locs;

	std::vector<LocStamped*> locs;

	BlobDetector blobDetector;
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
