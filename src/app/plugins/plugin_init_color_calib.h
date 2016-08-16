/*
 * plugin_init_color_calib.h
 *
 *  Created on: Aug 12, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef SRC_APP_PLUGINS_PLUGIN_INIT_COLOR_CALIB_H_
#define SRC_APP_PLUGINS_PLUGIN_INIT_COLOR_CALIB_H_

#include <visionplugin.h>
#include "cmvision_region.h"
#include "lut3d.h"
#include "field.h"
#include "camera_calibration.h"
#include "plugin_visualize.h"
#include "VarTypes.h"
#include "messages_robocup_ssl_detection.pb.h"

#include <chrono>

class PluginInitColorCalib: public VisionPlugin {
	Q_OBJECT
public:
	PluginInitColorCalib(FrameBuffer * _buffer, LUT3D * lut,
			const CameraParameters& camera_params, const RoboCupField& field);
	virtual ~PluginInitColorCalib();

	virtual ProcessResult process(FrameData * data, RenderOptions * options);
	virtual VarList * getSettings();
	virtual string getName();

protected slots:
	void slotUpdateTriggered();
private:
	virtual void classify();

	VarList * _settings;
	VarTrigger * _update;
	bool running;
	int nFrames;
	long nSamples;
	LUT3D * local_lut;
	LUT3D * global_lut;

	std::vector<int> clazz2Channel;
	std::vector<rgb> rgbColors;
	std::vector<yuv> yuvColors;
};

#endif /* SRC_APP_PLUGINS_PLUGIN_INIT_COLOR_CALIB_H_ */
