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

#include <chrono>
#include <lwpr.hh>

class LocStamped {
public:
	double time;
	pixelloc loc;
	int clazz;
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
private:
    virtual bool isInAnyRegion(CMVision::ColorRegionList * colorlist, pixelloc& loc, double maxArea, double margin);
    virtual void updateModel(FrameData * frame, pixelloc& loc, int clazz);
    virtual void CopytoLUT(LUT3D *lut);
    virtual void processRegion(FrameData* frame, CMVision::Region* region, int channel, float radius);
    LUT3D * _lut;
    VarList * _settings;
    VarBool * _v_enable;
    VarTrigger * _pub;

    const CameraParameters& camera_parameters;
    const RoboCupField& field;

	LWPR_Object* model;
	int last_nData = 0;
	std::deque<LocStamped> locs;
	std::chrono::time_point<std::chrono::system_clock> lastUpdate;
	std::vector<int> colorMap;
	int outDim;

};

#endif /* SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_ */
