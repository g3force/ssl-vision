//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    plugin_auto_color_calibration.h
  \brief   C++ Implementation: PluginAutoColorCalibration
  \author  Nicolai Ommer <nicolai.ommer@gmail.com>, (C) 2016
           Mark Geiger <markgeiger@posteo.de>, (C) 2017
*/
//========================================================================

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

#include <gui/automatedcolorcalibwidget.h>

#include "blob_detector.h"
#include "initial_color_calibrator.h"
#include "online_color_calibrator.h"
#include "color_calibrator.h"


class PluginAutoColorCalibration : public VisionPlugin {
Q_OBJECT

public:
    PluginAutoColorCalibration(FrameBuffer *_buffer,
                           LUT3D *lut,
                           const CameraParameters &camera_params,
                           const RoboCupField &field);

    ~PluginAutoColorCalibration() override;

    ProcessResult process(FrameData *data, RenderOptions *options) override;

    QWidget *getControlWidget() override;

    VarList *getSettings() override;

    string getName() override;

    void mousePressEvent(QMouseEvent *event, pixelloc loc) override;

    void mouseReleaseEvent(QMouseEvent *event, pixelloc loc) override;

    void mouseMoveEvent(QMouseEvent *event, pixelloc loc) override;

private:

    InitialColorCalibrator initialColorCalibrator;

    LUT3D *global_lut;
    const CameraParameters &camera_parameters;
    bool initial_calibration_running = false;
    int processed_frames = 0;

    AutomatedColorCalibWidget *_accw = nullptr;
    VarList *_settings;
    VarBool *_v_debug;
    OnlineColorCalibrator *onlineColorCalibrator;

    VarDouble *drag_x = nullptr;
    VarDouble *drag_y = nullptr;

    bool doing_drag = false;
    bool online_calibration_running = false;

    bool setDragParamsIfHit(pixelloc loc, VarDouble *x, VarDouble *y);

    void process_gui_commands();
};

#endif /* SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_ */
