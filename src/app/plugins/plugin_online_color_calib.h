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

class PluginOnlineColorCalib : public VisionPlugin
{
Q_OBJECT
public:
	PluginOnlineColorCalib(FrameBuffer * _buffer, LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field);
	virtual ~PluginOnlineColorCalib();

    virtual ProcessResult process(FrameData * data, RenderOptions * options);
    virtual VarList * getSettings();
    virtual string getName();

protected slots:
  void slotUpdateTriggered();
  void slotUpdateVisTriggered();
private:
    virtual bool isInAnyRegion(CMVision::ColorRegionList * colorlist, pixelloc& loc, double maxArea, double margin);
    virtual void updateModel(FrameData * frame, pixelloc& loc, int clazz);
    virtual void CopytoLUT(LUT3D *lut);
    virtual int getColorFromModelOutput(doubleVec& output);
    virtual void regionFieldDim(CMVision::Region* region, int clazz, double& width, double& height);
    virtual void regionDesiredPixelDim(CMVision::Region* region, int clazz, int& width, int& height);
    virtual void addRegionCross(
    		const SSL_DetectionFrame * detection_frame,
    		std::vector<LocStamped*>& locs,
    		int clazz,
    		int targetClazz,
    		CMVision::Region* region,
    		int width,
    		int height,
    		int exclWidth,
    		int exclHeight,
			int offset);
    virtual void addRegionEllipse(
    		const SSL_DetectionFrame * detection_frame,
    		std::vector<LocStamped*>& locs,
    		int clazz,
    		int targetClazz,
    		CMVision::Region* region,
    		int width,
    		int height,
    		int exclWidth,
    		int exclHeight,
			int offset);
    virtual bool isInAngleRange(vector3d& pField, int clazz, BotPosStamped* botPos);
    LUT3D * _lut;
    YUVLUT lut;
    VarList * _settings;
    VarBool * _v_enable;
    VarBool * _v_lifeUpdate;
    VarBool * _v_removeOutlierBlobs;
    VarTrigger * _updGlob;
    VarTrigger * _updVis;

    const CameraParameters& camera_parameters;
    const RoboCupField& field;

	LWPR_Object* model;

	std::vector<LocStamped*> locs;
	std::vector<BotPosStamped*> botPoss;
	std::vector<ClazzProperties> cProp;
};

#endif /* SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_ */
